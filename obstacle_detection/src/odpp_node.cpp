#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"

#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "nav2_dynamic_interface/msg/obstacle_array.hpp"
#include "nav2_dynamic_interface/msg/obstacle.hpp"
#include "nav2_dynamic_interface/srv/predict_obstacles.hpp"

#include "cluster_engine.hpp"
#include "tracker_engine.hpp"
#include "types.hpp"

/**
 * @class DynamicObstacleNode
 * @brief Main node for dynamic obstacle detection, tracking, and prediction
 * 
 * This node orchestrates:
 * - Costmap-based blob detection (via ClusterEngine)
 * - Multi-hypothesis tracking (via TrackerEngine with Hungarian matching)
 * - Future trajectory prediction (via external predictor service)
 * - Publishing of obstacle states for Nav2 consumption
 */
class DynamicObstacleNode : public rclcpp::Node
{
public:
  DynamicObstacleNode()
  : Node("dynamic_obstacle_node")
  {
    // Create separate callback groups for concurrent execution
    timer_cb_group_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    client_cb_group_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

    // Initialize costmap (lifecycle node)
    costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
      "dynamic_obstacle_costmap",
      std::string{get_namespace()},
      "dynamic_obstacle_costmap");

    rclcpp_lifecycle::State state;
    costmap_ros_->on_configure(state);
    costmap_ros_->on_activate(state);

    // Load tracking parameters
    double gate_distance = this->declare_parameter<double>("tracking.gate_distance", 1.5);
    int max_missed = this->declare_parameter<int>("tracking.max_missed", 3);
    future_len_ = this->declare_parameter<int>("prediction.future_len", 5);

    tracker_engine_.set_gate_distance(gate_distance);
    tracker_engine_.set_max_missed(max_missed);

    // Set fixed prediction interval (uniform timestep for future predictions)
    double pred_dt_sec = this->declare_parameter<double>("prediction.delta_time", 0.2);
    prediction_dt_.sec = static_cast<int32_t>(pred_dt_sec);
    prediction_dt_.nanosec = static_cast<uint32_t>(
      (pred_dt_sec - static_cast<double>(prediction_dt_.sec)) * 1e9);

    // Create service client for prediction
    predict_client_ = this->create_client<nav2_dynamic_interface::srv::PredictObstacles>(
      "predict_obstacles",
      rmw_qos_profile_services_default,
      client_cb_group_);

    // Create obstacle publisher
    obstacle_pub_ = this->create_publisher<nav2_dynamic_interface::msg::ObstacleArray>(
      "obstacles_array", 10);

    // Start periodic processing timer
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(200),
      std::bind(&DynamicObstacleNode::processCostmap, this),
      timer_cb_group_);

    RCLCPP_INFO(this->get_logger(), "Dynamic obstacle node initialized");
  }

  /**
   * @brief Get costmap node for executor registration
   * @return Shared pointer to Costmap2DROS node
   */
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> get_costmap_ros()
  {
    return costmap_ros_;
  }

private:
  // ROS 2 components
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::CallbackGroup::SharedPtr timer_cb_group_;
  rclcpp::CallbackGroup::SharedPtr client_cb_group_;
  rclcpp::Publisher<nav2_dynamic_interface::msg::ObstacleArray>::SharedPtr obstacle_pub_;
  rclcpp::Client<nav2_dynamic_interface::srv::PredictObstacles>::SharedPtr predict_client_;

  // Processing engines
  ClusterEngine cluster_engine_;
  TrackerEngine tracker_engine_;

  // Configuration
  int future_len_{5};  // Number of future prediction steps
  builtin_interfaces::msg::Duration prediction_dt_;  // Fixed prediction timestep

  // State
  std::unordered_map<int64_t, nav2_dynamic_interface::msg::Obstacle> cached_predictions_;

  /**
   * @brief Main processing loop: detect, track, predict, publish
   * 
   * Executes at fixed rate (200ms default):
   * 1. Extract blob clusters from costmap
   * 2. Update tracker with detections (Hungarian matching)
   * 3. Request predictions from external service
   * 4. Publish obstacle array with current + predicted states
   */
  void processCostmap()
  {
    auto costmap = costmap_ros_->getCostmap();
    const unsigned char cost_threshold = nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;

    // Step 1: Cluster detection
    std::vector<BlobCluster> clusters = cluster_engine_.extract(costmap, cost_threshold);

    // Step 2: Multi-hypothesis tracking
    std::vector<ObstacleTrack> tracks = tracker_engine_.update(clusters);

    rclcpp::Time now = this->get_clock()->now();

    // Step 3: Request future predictions
    publishPredictionRequest(tracks, prediction_dt_, now);

    // Step 4: Publish obstacle states
    publishObstacleArray(tracks, prediction_dt_, now, costmap_ros_->getGlobalFrameID());
  }

  /**
   * @brief Send prediction request to external service
   * 
   * @param tracks Current obstacle tracks from tracker
   * @param dt_prediction Fixed timestep for prediction spacing
   * @param stamp Current timestamp
   * 
   * Sends current obstacle states to predictor service, receives future
   * trajectories, and updates tracker's predicted positions for next cycle.
   */
  void publishPredictionRequest(
    const std::vector<ObstacleTrack> & tracks,
    const builtin_interfaces::msg::Duration & dt_prediction,
    const rclcpp::Time & stamp)
  {
    using namespace std::chrono_literals;

    if (!predict_client_ || !predict_client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Prediction service not ready");
      return;
    }

    if (tracks.empty()) {
      RCLCPP_DEBUG(get_logger(), "No tracks to predict");
      return;
    }

    // Build prediction request
    auto req = std::make_shared<nav2_dynamic_interface::srv::PredictObstacles::Request>();
    req->pre_prediction_header.stamp = stamp;
    req->pre_prediction_header.frame_id = costmap_ros_->getGlobalFrameID();
    req->prediction_steps = future_len_;

    for (const auto & track : tracks) {
      nav2_dynamic_interface::msg::Obstacle ob;
      ob.header.stamp = stamp;
      ob.header.frame_id = costmap_ros_->getGlobalFrameID();
      ob.id = track.id;
      ob.uuid = track.uuid;
      ob.delta_time = dt_prediction;

      // Current pose (predictor maintains history internally)
      geometry_msgs::msg::PoseWithCovariance current_pose;
      current_pose.pose.position = track.current_position;
      current_pose.pose.orientation.w = 1.0;
      ob.position.push_back(current_pose);

      ob.polygon = track.polygon;
      req->obstacles_past.push_back(std::move(ob));
    }

    // Call service (async with timeout)
    auto future = predict_client_->async_send_request(req);
    if (future.wait_for(150ms) != std::future_status::ready) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Prediction service timeout");
      return;
    }

    auto resp = future.get();

    // Extract predictions
    std::unordered_map<int64_t, geometry_msgs::msg::Point> predicted_t1;
    for (auto & pred_ob : resp->obstacles_future) {
      if (!pred_ob.position.empty()) {
        predicted_t1[pred_ob.id] = pred_ob.position[0].pose.position;
      }
      cached_predictions_[pred_ob.id] = pred_ob;
    }

    // Update tracker's expectations for next cycle
    tracker_engine_.updatePredictedPositions(predicted_t1);

    RCLCPP_DEBUG(get_logger(), "Prediction processed successfully");
  }

  /**
   * @brief Publish obstacle array with current + predicted states
   * 
   * @param tracks Current obstacle tracks
   * @param dt Prediction timestep
   * @param stamp Current timestamp
   * @param frame_id Reference frame
   * 
   * Combines current detections with cached predictions to produce
   * complete obstacle trajectories for Nav2 planners.
   */
  void publishObstacleArray(
    const std::vector<ObstacleTrack> & tracks,
    const builtin_interfaces::msg::Duration & dt,
    const rclcpp::Time & stamp,
    const std::string & frame_id)
  {
    nav2_dynamic_interface::msg::ObstacleArray msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id;

    for (const auto & track : tracks) {
      nav2_dynamic_interface::msg::Obstacle ob;
      ob.header.stamp = stamp;
      ob.header.frame_id = frame_id;
      ob.id = track.id;
      ob.uuid = track.uuid;
      ob.delta_time = dt;

      // Current pose
      geometry_msgs::msg::PoseWithCovariance current_pose;
      current_pose.pose.position = track.current_position;
      current_pose.pose.orientation.w = 1.0;
      ob.position.push_back(current_pose);

      // Append cached predictions (future poses + velocities)
      auto pred_it = cached_predictions_.find(track.id);
      if (pred_it != cached_predictions_.end()) {
        ob.position.insert(
          ob.position.end(),
          pred_it->second.position.begin(),
          pred_it->second.position.end());
        ob.velocity = pred_it->second.velocity;
      }

      ob.polygon = track.polygon;
      msg.obstacles.push_back(std::move(ob));
    }

    obstacle_pub_->publish(msg);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<DynamicObstacleNode>();

  // Multi-threaded executor for concurrent callback execution
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
  executor.add_node(node);

  // Add costmap node to executor
  auto costmap_node = node->get_costmap_ros();
  executor.add_node(costmap_node->get_node_base_interface());

  executor.spin();

  rclcpp::shutdown();
  return 0;
}
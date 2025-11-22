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
#include "nav2_dynamic_interface/msg/obstacle.hpp"
#include "nav2_dynamic_interface/msg/obstacle_array.hpp"
#include "nav2_dynamic_interface/msg/obstacle_sequence_array.hpp"


#include "cluster_engine.hpp"
#include "tracker_engine.hpp"
#include "types.hpp"

/**
 * @class DetectionNode
 * @brief Main node for dynamic obstacle detection and tracking
 *
 * This node orchestrates:
 * - Costmap-based blob detection (via ClusterEngine)
 * - Multi-hypothesis tracking (via TrackerEngine with Hungarian matching)
 * - Publishing current detections to predictor
 * - Receiving predicted trajectories to update tracker state
 */
class DetectionNode : public rclcpp::Node
{
public:
  DetectionNode()
  : Node("detection_node")
  {
    // Create separate callback groups for concurrent execution
    timer_cb_group_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    sub_cb_group_ = this->create_callback_group(
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

    tracker_engine_.set_gate_distance(gate_distance);
    tracker_engine_.set_max_missed(max_missed);

    // Set fixed prediction interval (uniform timestep for future predictions)
    double pred_dt_sec = this->declare_parameter<double>("prediction.delta_time", 0.2);
    prediction_dt_.sec = static_cast<int32_t>(pred_dt_sec);
    prediction_dt_.nanosec = static_cast<uint32_t>(
      (pred_dt_sec - static_cast<double>(prediction_dt_.sec)) * 1e9);

    // Publisher: send detections to predictor
    detections_pub_ = this->create_publisher<nav2_dynamic_interface::msg::ObstacleArray>(
      "obstacles_detected", 10);

    // Subscriber: receive predictions from predictor
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = sub_cb_group_;
    predictions_sub_ = this->create_subscription<nav2_dynamic_interface::msg::ObstacleSequenceArray>(
      "obstacles_sequences", 10,
      std::bind(&DetectionNode::onPredictions, this, std::placeholders::_1),
      sub_options);

    // Start periodic processing timer
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(200),
      std::bind(&DetectionNode::processCostmap, this),
      timer_cb_group_);

    RCLCPP_INFO(this->get_logger(), "Detection node initialized");
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
  rclcpp::CallbackGroup::SharedPtr sub_cb_group_;
  rclcpp::Publisher<nav2_dynamic_interface::msg::ObstacleArray>::SharedPtr detections_pub_;
  rclcpp::Subscription<nav2_dynamic_interface::msg::ObstacleSequenceArray>::SharedPtr predictions_sub_;

  // Processing engines
  ClusterEngine cluster_engine_;
  TrackerEngine tracker_engine_;

  // Configuration
  builtin_interfaces::msg::Duration prediction_dt_;  // Fixed prediction timestep

  // State
  std::unordered_map<int64_t, nav2_dynamic_interface::msg::Obstacle> cached_predictions_;

  /**
   * @brief Main processing loop: detect, track, publish detections
   *
   * Executes at fixed rate (200ms default):
   * 1. Extract blob clusters from costmap
   * 2. Update tracker with detections (Hungarian matching)
   * 3. Publish current detections to predictor
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

    // Step 3: Publish detections to predictor
    publishDetections(tracks, prediction_dt_, now);
  }

  /**
   * @brief Publish current detections to predictor
   *
   * @param tracks Current obstacle tracks from tracker
   * @param dt_prediction Fixed timestep for prediction spacing
   * @param stamp Current timestamp
   *
   * Publishes current obstacle states on "obstacles_detected" topic.
   * Predictor will process and publish predictions on "obstacles_predicted".
   */
  void publishDetections(
    const std::vector<ObstacleTrack> & tracks,
    const builtin_interfaces::msg::Duration & dt_prediction,
    const rclcpp::Time & stamp)
  {
    if (tracks.empty()) {
      RCLCPP_DEBUG(get_logger(), "No tracks to publish");
      return;
    }

    nav2_dynamic_interface::msg::ObstacleArray msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = costmap_ros_->getGlobalFrameID();

    for (const ObstacleTrack & track : tracks) {
      nav2_dynamic_interface::msg::Obstacle ob;
      ob.header.stamp = stamp;
      ob.header.frame_id = costmap_ros_->getGlobalFrameID();
      ob.id = track.id;
      ob.uuid = track.uuid;

      // Current pose (predictor maintains history internally)
      geometry_msgs::msg::PoseWithCovariance current_pose;
      current_pose.pose.position = track.current_position;
      current_pose.pose.orientation.w = 1.0;
      ob.position = current_pose;

      ob.polygon = track.polygon;
      msg.obstacles.push_back(std::move(ob));
    }

    detections_pub_->publish(msg);

    RCLCPP_DEBUG(get_logger(), "Published %zu detections", tracks.size());
  }

  /**
   * @brief Callback for predictions published by predictor
   *
   * @param msg ObstacleArray containing current + predicted positions
   *
   * Updates tracker's predicted positions for next cycle's Hungarian matching.
   * Caches predictions for potential later use.
   */
  void onPredictions(const nav2_dynamic_interface::msg::ObstacleSequenceArray::SharedPtr msg)
  {
    std::unordered_map<int64_t, geometry_msgs::msg::Point> predicted_t1;

    for (nav2_dynamic_interface::msg::ObstacleSequence &sequence : msg->obstacle_sequences) {
      // Extract t+1 prediction (position[1] if available, else fallback to position[0])
      nav2_dynamic_interface::msg::Obstacle obstacle;
      obstacle.id = sequence.id;
      obstacle.header = sequence.header;
      
      if (sequence.positions.size() > 1) { // there is at least one prediction step
        predicted_t1[sequence.id] = sequence.positions[1].pose.position;
        obstacle.position = sequence.positions[1];
      } else{
        // Fallback: use current position if no prediction available
        predicted_t1[sequence.id] = sequence.positions[0].pose.position;
        obstacle.position = sequence.positions[0];
      }

      // Cache full obstacle data
      cached_predictions_[sequence.id] = obstacle;
    }

    // Update tracker's expectations for next cycle
    tracker_engine_.updatePredictedPositions(predicted_t1);

    RCLCPP_DEBUG(
      get_logger(),
      "Updated tracker with predictions for %zu obstacles",
      predicted_t1.size());
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<DetectionNode>();

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
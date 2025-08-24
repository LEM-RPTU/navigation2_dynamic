#include "rclcpp/rclcpp.hpp"
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>

#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_costmap_2d/cost_values.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <chrono>
#include <memory>
#include <thread>
#include <vector>
#include <string>
#include <cmath>

#include "nav2_dynamic_msgs/msg/obstacle_array.hpp"
#include "nav2_dynamic_msgs/msg/obstacle.hpp"
#include "nav2_dynamic_msgs/srv/predict_obstacles.hpp"
#include "cluster_engine.hpp"
#include "tracker_engine.hpp"
#include "types.hpp"

class DynamicObstacleNode : public rclcpp::Node
{
public:
    DynamicObstacleNode()
        : Node("dynamic_obstacle_node")
    {
        // Create separate callback groups
        timer_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        client_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        // Bring up Costmap2DROS (lifecycle)
        costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
            "dynamic_obstacle_costmap", std::string{get_namespace()}, "dynamic_obstacle_costmap");

        // No separate thread - will be managed by the executor
        rclcpp_lifecycle::State state;
        costmap_ros_->on_configure(state);
        costmap_ros_->on_activate(state);

        // Params
        double gate = this->declare_parameter<double>("tracking.gate_distance", 1.5);
        int max_missed = this->declare_parameter<int>("tracking.max_missed", 3);
        future_len_ = this->declare_parameter<int>("prediction.future_len", 5);

        tracker_engine_.set_gate_distance(gate);
        tracker_engine_.set_max_missed(max_missed);

        // Publishers
        predict_client_ = this->create_client<nav2_dynamic_msgs::srv::PredictObstacles>(
            "predict_obstacles", rmw_qos_profile_services_default, client_cb_group_);

        obstacle_pub_ = this->create_publisher<nav2_dynamic_msgs::msg::ObstacleArray>("obstacles_array", 10);

        // Periodic processing
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(200),
            std::bind(&DynamicObstacleNode::processCostmap, this),
            timer_cb_group_);

        RCLCPP_INFO(this->get_logger(), "Dynamic obstacle node started");
    }

    // Helper to add costmap node to executor in main
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> get_costmap_ros()
    {
        return costmap_ros_;
    }

private:
    // State
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::CallbackGroup::SharedPtr timer_cb_group_;
    rclcpp::CallbackGroup::SharedPtr client_cb_group_;
    rclcpp::Publisher<nav2_dynamic_msgs::msg::ObstacleArray>::SharedPtr obstacle_pub_;
    rclcpp::Client<nav2_dynamic_msgs::srv::PredictObstacles>::SharedPtr predict_client_;

    ClusterEngine cluster_engine_;
    TrackerEngine tracker_engine_;
    int future_len_{5};

    void processCostmap()
    {
        auto costmap = costmap_ros_->getCostmap();
        if (!costmap)
            return;

        const unsigned char cost_threshold = nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;

        // 1) Clustering
        std::vector<BlobCluster> clusters = cluster_engine_.extract(costmap, cost_threshold);

        // 2) Tracking with TrackerPredictionFrame
        TrackerPredictionFrame tr = tracker_engine_.update(clusters);

        // 3) Publish prediction request
        publishPredictionRequest(tr);

        // 4) Publish obstacle array (using track data directly)
        publishObstacleArray(tr, costmap_ros_->getGlobalFrameID());
    }

    void publishPredictionRequest(TrackerPredictionFrame &tr)
    {
        using namespace std::chrono_literals;

        if (!predict_client_ || !predict_client_->service_is_ready())
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Prediction service not available");
            return;
        }

        // Skip if no tracks (no clusters / nothing to predict)
        if (tr.tracks.empty())
        {
            RCLCPP_DEBUG(this->get_logger(), "Skip prediction: no tracks present");
            return;
        }

        auto req = std::make_shared<nav2_dynamic_msgs::srv::PredictObstacles::Request>();
        req->pre_prediction_header.stamp = tr.stamp;
        req->pre_prediction_header.frame_id = costmap_ros_->getGlobalFrameID();
        req->prediction_steps = future_len_;
        req->obstacles_past.reserve(tr.tracks.size());

        for (const auto &track : tr.tracks)
        {
            nav2_dynamic_msgs::msg::Obstacle ob;
            ob.id = track.id;
            ob.uuid = track.uuid;
            ob.position.clear();
            ob.position.reserve(track.history.size());
            for (size_t i = 0; i < track.history.size(); ++i)
            {
                ob.position.push_back(track.history[i]);
            }
            ob.velocity = track.velocity;
            ob.heading = track.heading;
            ob.polygon = track.polygon;
            ob.position_covariance = track.position_covariance;
            ob.velocity_covariance = track.velocity_covariance;

            req->obstacles_past.push_back(std::move(ob));
        }

        auto future = predict_client_->async_send_request(req);

        RCLCPP_DEBUG(this->get_logger(), "Waiting for prediction response...");

        // Use direct wait_for instead of spin_until_future_complete
        // This avoids the "already added to an executor" error
        if (future.wait_for(150ms) != std::future_status::ready)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Prediction request timed out");
            return;
        }

        auto resp = future.get();

        // Integrate predictions into current tracks by ID (later switch to UUID)
        for (auto &pred_ob : resp->obstacles_future)
        {

            auto it = std::find_if(tr.tracks.begin(), tr.tracks.end(),
                                   [&](auto &trk)
                                   { return trk.id == pred_ob.id; });
            if (it == tr.tracks.end())
                continue;

            // Assume pred_ob.position holds ONLY future predicted points (not including current).
            // Prepend current measured position if we have one in track.position already.
            // If track.position[0] already current, replace rest with predictions.
            if (!it->position.empty())
            {
                std::vector<geometry_msgs::msg::Point> merged;
                merged.reserve(1 + pred_ob.position.size());
                merged.push_back(it->position.front()); // current
                merged.insert(merged.end(), pred_ob.position.begin(), pred_ob.position.end());
                it->position = std::move(merged);
            }
            else
            {
                // No current position stored; just take predictions
                it->position = pred_ob.position;
            }

            // Optionally update kinematics if provided
            if (pred_ob.velocity.x != 0.0 || pred_ob.velocity.y != 0.0 || pred_ob.velocity.z != 0.0)
                it->velocity = pred_ob.velocity;
            it->heading = pred_ob.heading;
            // Covariances
            if (pred_ob.position_covariance != it->position_covariance)
            {
                it->position_covariance = pred_ob.position_covariance;
            }
            if (pred_ob.velocity_covariance != it->velocity_covariance)
            {
                it->velocity_covariance = pred_ob.velocity_covariance;
            }
        }

        RCLCPP_INFO(this->get_logger(), "Prediction response processed successfully");
    }

    void publishObstacleArray(const TrackerPredictionFrame &tr, const std::string &frame_id)
    {
        nav2_dynamic_msgs::msg::ObstacleArray msg;
        msg.header.stamp = tr.stamp;
        msg.header.frame_id = frame_id;
        msg.obstacles.reserve(tr.tracks.size());

        for (const auto &track : tr.tracks)
        {
            nav2_dynamic_msgs::msg::Obstacle ob;

            // ID and score
            ob.id = track.id;

            // UUID - convert from int64 to UUID if needed
            ob.uuid = track.uuid;

            // Positions - current + predictions
            ob.position = track.position;
            // Ensure at least current + future_len positions
            if (ob.position.size() < static_cast<size_t>(future_len_ + 1))
            {
                const auto &current = ob.position.empty() ? geometry_msgs::msg::Point() : ob.position.front();

                while (ob.position.size() <= static_cast<size_t>(future_len_))
                {
                    ob.position.push_back(current);
                }
            }

            // Kinematics
            ob.velocity = track.velocity;
            ob.heading = track.heading;

            // Polygon hull
            ob.polygon = track.polygon;

            // Covariances
            ob.position_covariance = track.position_covariance;
            ob.velocity_covariance = track.velocity_covariance;

            msg.obstacles.push_back(std::move(ob));
        }

        obstacle_pub_->publish(msg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DynamicObstacleNode>();

    // Add both nodes to the executor
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
    executor.add_node(node);

    // Add costmap node to the executor
    auto costmap_node = node->get_costmap_ros();
    executor.add_node(costmap_node->get_node_base_interface());

    executor.spin();

    rclcpp::shutdown();
    return 0;
}
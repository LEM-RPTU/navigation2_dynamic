#include "rclcpp/rclcpp.hpp"
#include <rclcpp_lifecycle/lifecycle_node.hpp>
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
#include "nav2_dynamic_msgs/msg/prediction_request.hpp"
#include "cluster_engine.hpp"
#include "tracker_engine.hpp"
#include "types.hpp"

class DynamicObstacleNode : public rclcpp::Node
{
public:
    DynamicObstacleNode()
        : Node("dynamic_obstacle_node")
    {
        // Bring up Costmap2DROS (lifecycle)
        costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
            "dynamic_obstacle_costmap", std::string{get_namespace()}, "dynamic_obstacle_costmap");

        costmap_thread_ = std::make_unique<std::thread>(
            [](rclcpp_lifecycle::LifecycleNode::SharedPtr node)
            {
                rclcpp::spin(node->get_node_base_interface());
            },
            costmap_ros_);

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
        predict_req_pub_ = this->create_publisher<nav2_dynamic_msgs::msg::PredictionRequest>("predict_request", 10);

        obstacle_pub_ = this->create_publisher<nav2_dynamic_msgs::msg::ObstacleArray>("obstacles_array", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("cluster_markers", 10);

        // Periodic processing
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(200),
            std::bind(&DynamicObstacleNode::processCostmap, this));

        RCLCPP_INFO(this->get_logger(), "DynamicObstacleNode up (cluster → track → publish).");
    }

private:
    // State
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
    std::unique_ptr<std::thread> costmap_thread_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<nav2_dynamic_msgs::msg::ObstacleArray>::SharedPtr obstacle_pub_;
    rclcpp::Publisher<nav2_dynamic_msgs::msg::PredictionRequest>::SharedPtr predict_req_pub_;

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

        // 4) Receive Prediction - TODO: implement callback

        // 5) Publish obstacle array (using track data directly)
        publishObstacleArray(tr, costmap_ros_->getGlobalFrameID());

        // 6) Visualization
        publishClusterMarkers(clusters, tr, costmap_ros_->getGlobalFrameID());
    }

    void publishPredictionRequest(const TrackerPredictionFrame &tr)
    {
        nav2_dynamic_msgs::msg::PredictionRequest msg;
        msg.prediction_steps = future_len_;
        msg.obstacles.reserve(tr.tracks.size());

        for (const auto &track : tr.tracks)
        {
            nav2_dynamic_msgs::msg::Obstacle tr_ob;
            tr_ob.id = track.history.size();
            // tr_ob.uuid = track.uuid;
            // tr_ob.velocity = track.velocity;
            // tr_ob.heading = track.heading;

            // Convert history to chronological order (oldest→newest)
            // Since TrackerEngine stores history with most-recent at the back
            tr_ob.position.clear();
            tr_ob.position.reserve(track.history.size());

            // History is stored most-recent at back; send oldest -> newest

            for (size_t i = 0; i < track.history.size(); ++i)
            {
                tr_ob.position.push_back(track.history[i]);
            }

            msg.obstacles.push_back(std::move(tr_ob));
        }

        predict_req_pub_->publish(msg);
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

    void publishClusterMarkers(const std::vector<BlobCluster> &clusters,
                               const TrackerPredictionFrame &tr,
                               const std::string &frame_id)
    {
        // Build lookup from cluster index to track ID
        std::vector<int64_t> ids(clusters.size(), -1);
        for (const auto &a : tr.assignments)
        {
            if (a.cluster_index < ids.size())
                ids[a.cluster_index] = a.id;
        }

        visualization_msgs::msg::MarkerArray ma;
        rclcpp::Time stamp = tr.stamp;

        // Clear previous markers
        visualization_msgs::msg::Marker clear;
        clear.header.frame_id = frame_id;
        clear.header.stamp = stamp;
        clear.ns = "clusters";
        clear.id = 0;
        clear.action = visualization_msgs::msg::Marker::DELETEALL;
        ma.markers.push_back(clear);

        int id = 1;

        for (size_t i = 0; i < clusters.size(); ++i)
        {
            const auto &c = clusters[i];

            // Hull
            visualization_msgs::msg::Marker hull;
            hull.header.frame_id = frame_id;
            hull.header.stamp = stamp;
            hull.ns = "clusters_hull";
            hull.id = id++;
            hull.type = visualization_msgs::msg::Marker::LINE_STRIP;
            hull.action = visualization_msgs::msg::Marker::ADD;
            hull.scale.x = 0.03;
            hull.color.r = 1.0f;
            hull.color.g = 0.3f;
            hull.color.b = 0.1f;
            hull.color.a = 0.9f;
            hull.lifetime = rclcpp::Duration(0, 7e8);
            hull.points = c.boundary;
            if (c.boundary.size() >= 3)
                hull.points.push_back(c.boundary.front());
            hull.frame_locked = true;
            ma.markers.push_back(std::move(hull));

            // Centroid
            visualization_msgs::msg::Marker centroid;
            centroid.header.frame_id = frame_id;
            centroid.header.stamp = stamp;
            centroid.ns = "clusters_centroid";
            centroid.id = id++;
            centroid.type = visualization_msgs::msg::Marker::SPHERE;
            centroid.action = visualization_msgs::msg::Marker::ADD;
            centroid.scale.x = 0.12;
            centroid.scale.y = 0.12;
            centroid.scale.z = 0.12;
            centroid.color.r = 0.2f;
            centroid.color.g = 1.0f;
            centroid.color.b = 0.2f;
            centroid.color.a = 0.95f;
            centroid.pose.position = c.centroid;
            centroid.pose.orientation.w = 1.0;
            centroid.lifetime = rclcpp::Duration(0, 7e8);
            centroid.frame_locked = true;
            ma.markers.push_back(std::move(centroid));

            // Label with ID
            visualization_msgs::msg::Marker label;
            label.header.frame_id = frame_id;
            label.header.stamp = stamp;
            label.ns = "clusters_label";
            label.id = id++;
            label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            label.action = visualization_msgs::msg::Marker::ADD;
            label.scale.z = 0.25;
            label.color.r = 1.0f;
            label.color.g = 0.9f;
            label.color.b = 0.1f;
            label.color.a = 1.0f;
            label.lifetime = rclcpp::Duration(0, 7e8);
            label.pose.position = c.centroid;
            label.pose.position.z += 0.3;
            label.pose.orientation.w = 1.0;
            label.text = (ids[i] >= 0) ? ("ID " + std::to_string(ids[i])) : "ID ?";
            label.frame_locked = true;
            ma.markers.push_back(std::move(label));
        }

        marker_pub_->publish(ma);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DynamicObstacleNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
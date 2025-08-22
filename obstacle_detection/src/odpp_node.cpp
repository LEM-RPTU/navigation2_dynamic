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
#include <unordered_map>
#include <future>
#include <mutex>
#include <condition_variable>

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
        obstacle_pub_ = this->create_publisher<nav2_dynamic_msgs::msg::ObstacleArray>("obstacles_array", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("cluster_markers", 10);

        // Service client
        predict_req_client_ = this->create_client<nav2_dynamic_msgs::srv::PredictObstacles>("predict_obstacles");

        // Periodic processing
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&DynamicObstacleNode::processCostmap, this));

        RCLCPP_INFO(this->get_logger(), "DynamicObstacleNode up (multi-threaded)");
    }

    ~DynamicObstacleNode()
    {
    }

private:
    // State
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
    std::unique_ptr<std::thread> costmap_thread_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<nav2_dynamic_msgs::msg::ObstacleArray>::SharedPtr obstacle_pub_;
    rclcpp::Client<nav2_dynamic_msgs::srv::PredictObstacles>::SharedPtr predict_req_client_;

    ClusterEngine cluster_engine_;
    TrackerEngine tracker_engine_;
    bool processing_{false};
    bool prediction_success_{false};
    rclcpp::Time tick_start_;
    int future_len_{5};
    int prediction_timeout_ms_{500};

    // Multithreading support
    // std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
    // std::thread executor_thread_;
    
    // Frame synchronization
    std::mutex frame_mutex_;
    std::condition_variable frame_cv_;
    bool prediction_in_progress_{false};
    TrackerPredictionFrame latest_frame_;
    
    void processCostmap()
    {
        auto costmap = costmap_ros_->getCostmap();
        if (!costmap) return;

        tick_start_ = this->now();
        RCLCPP_INFO(this->get_logger(), "Processing new costmap data");

        // Cluster and track (these are CPU intensive)
        std::vector<BlobCluster> clusters = cluster_engine_.extract(costmap, nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
        TrackerPredictionFrame tr = tracker_engine_.update(clusters);
        tr.stamp = this->now();

        // Update latest frame and start prediction asynchronously
        {
            std::lock_guard<std::mutex> lock(frame_mutex_);
            latest_frame_ = tr;
        }
        
        // Always publish immediately with current data
        publishObstacleArray(tr, costmap_ros_->getGlobalFrameID());
        publishClusterMarkers(tr, costmap_ros_->getGlobalFrameID());
        
        // Asynchronously request prediction for next cycle
        std::thread([this, tr]() {
            requestPredictionAsync(tr);
        }).detach();

        // Complete cycle timing
        rclcpp::Duration dt = this->now() - tick_start_;
        RCLCPP_INFO(this->get_logger(), "Costmap processing complete (%.3f s)", dt.seconds());
    }

    void requestPredictionAsync(TrackerPredictionFrame tr)
    {
        if (!predict_req_client_->service_is_ready()) {
            RCLCPP_WARN(this->get_logger(), "Prediction service not ready");
            return;
        }

        auto request = std::make_shared<nav2_dynamic_msgs::srv::PredictObstacles::Request>();
        request->prediction_steps = future_len_;
        request->pre_prediction_header.stamp = tr.stamp;
        request->pre_prediction_header.frame_id = costmap_ros_->getGlobalFrameID();

        for (const auto &track : tr.tracks) {
            nav2_dynamic_msgs::msg::Obstacle tr_ob;
            tr_ob.id = track.id;
            tr_ob.uuid = track.uuid;
            tr_ob.position.reserve(track.history.size());
            for (const auto &point : track.history) {
                tr_ob.position.push_back(point);
            }
            request->obstacles_past.push_back(std::move(tr_ob));
        }

        // Use future with timeout
        auto start_time = this->now();
        auto result_future = predict_req_client_->async_send_request(
            request,
            [this, tr, start_time](
                rclcpp::Client<nav2_dynamic_msgs::srv::PredictObstacles>::SharedFuture future) {
                handlePredictionResponse(future, tr, start_time);
            });
    }

    void handlePredictionResponse(
        rclcpp::Client<nav2_dynamic_msgs::srv::PredictObstacles>::SharedFuture future,
        TrackerPredictionFrame original_frame,
        rclcpp::Time start_time)
    {
        if (future.valid()) {
            auto response = future.get();
            auto dt = this->now() - start_time;
            RCLCPP_INFO(this->get_logger(), "Prediction received after %.3fs", dt.seconds());
            
            // Find latest frame to update with predictions
            std::unique_lock<std::mutex> lock(frame_mutex_);
            updateTracksWithPrediction(latest_frame_, response->obstacles_future);
            
            // Republish with predictions
            publishObstacleArray(latest_frame_, costmap_ros_->getGlobalFrameID());
            publishClusterMarkers(latest_frame_, costmap_ros_->getGlobalFrameID());
        }
    }

    void updateTracksWithPrediction(TrackerPredictionFrame &tr,
                                    const std::vector<nav2_dynamic_msgs::msg::Obstacle> &predicted_obstacles)
    {
        // Map of track id to index in tr.tracks for quick lookup
        std::unordered_map<int64_t, size_t> track_index_map;
        for (size_t i = 0; i < tr.tracks.size(); ++i)
        {
            track_index_map[tr.tracks[i].id] = i;
        }

        // Update tracks with prediction data
        for (const auto &pred_obstacle : predicted_obstacles)
        {
            auto it = track_index_map.find(pred_obstacle.id);
            if (it == track_index_map.end())
            {
                continue; // Skip obstacles not in our tracking list
            }

            ObstacleTrack &track = tr.tracks[it->second];

            // Update position array (current + future predictions)
            if (!pred_obstacle.position.empty())
            {
                // Keep the current position and add predictions
                if (!track.position.empty())
                {
                    // First position is current, keep it
                    geometry_msgs::msg::Point current = track.position.front();

                    // Replace with predicted future positions
                    track.position.clear();
                    track.position.push_back(current); // Keep current position

                    // Add predictions
                    for (size_t i = 0; i < pred_obstacle.position.size(); ++i)
                    {
                        track.position.push_back(pred_obstacle.position[i]);
                    }
                }
                else
                {
                    // If no current position, use predictions directly
                    track.position = pred_obstacle.position;
                }
            }

            // Update kinematics if provided
            if (pred_obstacle.velocity.x != 0 || pred_obstacle.velocity.y != 0 || pred_obstacle.velocity.z != 0)
            {
                track.velocity = pred_obstacle.velocity;
            }

            if (pred_obstacle.heading.x != 0 || pred_obstacle.heading.y != 0 || pred_obstacle.heading.z != 0)
            {
                track.heading = pred_obstacle.heading;
            }

            // Update position MSE
            if (!track.position.empty() && !pred_obstacle.position.empty())
            {
                track.position_covariance = pred_obstacle.position_covariance;
                track.velocity_covariance = pred_obstacle.velocity_covariance;
            }
        }
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

    void publishClusterMarkers(const TrackerPredictionFrame &tr, const std::string &frame_id)
    {
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

        for (const auto &t : tr.tracks)
        {
            // Current position (position[0] = detection or prediction)
            geometry_msgs::msg::Point current{};
            if (!t.position.empty())
            {
                current = t.position.front();
            }

            // Hull from track.polygon (Point32 -> Point)
            std::vector<geometry_msgs::msg::Point> hull_points;
            hull_points.reserve(t.polygon.points.size() + 1);
            for (const auto &p32 : t.polygon.points)
            {
                geometry_msgs::msg::Point p;
                p.x = p32.x;
                p.y = p32.y;
                p.z = p32.z;
                hull_points.push_back(p);
            }
            if (hull_points.size() >= 3)
            {
                hull_points.push_back(hull_points.front()); // close loop
            }

            // Hull marker
            visualization_msgs::msg::Marker hull;
            hull.header.frame_id = frame_id;
            hull.header.stamp = stamp;
            hull.ns = "clusters_hull";
            hull.id = t.id;
            hull.type = visualization_msgs::msg::Marker::LINE_STRIP;
            hull.action = visualization_msgs::msg::Marker::ADD;
            hull.scale.x = 0.03;
            hull.color.r = 1.0f;
            hull.color.g = 0.3f;
            hull.color.b = 0.1f;
            hull.color.a = 0.9f;
            hull.lifetime = rclcpp::Duration(0, 7e8);
            hull.points = std::move(hull_points);
            hull.frame_locked = true;
            ma.markers.push_back(std::move(hull));

            // Centroid marker (current track position)
            visualization_msgs::msg::Marker centroid;
            centroid.header.frame_id = frame_id;
            centroid.header.stamp = stamp;
            centroid.ns = "clusters_centroid";
            centroid.id = t.id;
            centroid.type = visualization_msgs::msg::Marker::SPHERE;
            centroid.action = visualization_msgs::msg::Marker::ADD;
            centroid.scale.x = 0.12;
            centroid.scale.y = 0.12;
            centroid.scale.z = 0.12;
            centroid.color.r = 0.2f;
            centroid.color.g = 1.0f;
            centroid.color.b = 0.2f;
            centroid.color.a = 0.95f;
            centroid.pose.position = current;
            centroid.pose.orientation.w = 1.0;
            centroid.lifetime = rclcpp::Duration(0, 7e8);
            centroid.frame_locked = true;
            ma.markers.push_back(std::move(centroid));

            // Label (track ID)
            visualization_msgs::msg::Marker label;
            label.header.frame_id = frame_id;
            label.header.stamp = stamp;
            label.ns = "clusters_label";
            label.id = t.id;
            label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            label.action = visualization_msgs::msg::Marker::ADD;
            label.scale.z = 0.25;
            label.color.r = 1.0f;
            label.color.g = 0.9f;
            label.color.b = 0.1f;
            label.color.a = 1.0f;
            label.lifetime = rclcpp::Duration(0, 7e8);
            label.pose.position = current;
            label.pose.position.z += 0.3;
            label.pose.orientation.w = 1.0;
            label.text = std::to_string(t.id);
            label.frame_locked = true;
            ma.markers.push_back(std::move(label));

            // Add covariance ellipse visualization
            if (t.position_covariance[0] > 0.0 && t.position_covariance[3] > 0.0) {
                visualization_msgs::msg::Marker cov_ellipse;
                cov_ellipse.header.frame_id = frame_id;
                cov_ellipse.header.stamp = stamp;
                cov_ellipse.ns = "clusters_covariance";
                cov_ellipse.id = t.id;
                cov_ellipse.type = visualization_msgs::msg::Marker::LINE_STRIP;
                cov_ellipse.action = visualization_msgs::msg::Marker::ADD;
                cov_ellipse.scale.x = 0.025;  // line width
                cov_ellipse.color.r = 0.95f;
                cov_ellipse.color.g = 0.95f;
                cov_ellipse.color.b = 0.10f;
                cov_ellipse.color.a = 0.85f;
                cov_ellipse.lifetime = rclcpp::Duration(0, 7e8);
                cov_ellipse.frame_locked = true;

                // Place ellipse at centroid
                cov_ellipse.pose.position = current;

                // Cov matrix (row-major 2x2)
                double xx = t.position_covariance[0];
                double xy = t.position_covariance[1];
                double yx = t.position_covariance[2];
                double yy = t.position_covariance[3];

                // Force symmetry (defensive)
                double cxy = 0.5 * (xy + yx);

                // Eigenvalues
                double trace = xx + yy;
                double det = xx * yy - cxy * cxy;
                if (det < 0.0) det = 0.0;
                double disc = trace * trace - 4.0 * det;
                if (disc < 0.0) disc = 0.0;
                double sqrt_disc = std::sqrt(disc);
                double l1 = 0.5 * (trace + sqrt_disc);
                double l2 = 0.5 * (trace - sqrt_disc);
                if (l1 < 1e-9) l1 = 1e-9;
                if (l2 < 1e-9) l2 = 1e-9;

                // Principal axis angle (atan2 for symmetric 2x2)
                double theta = 0.0;
                if (std::fabs(cxy) > 1e-9 || std::fabs(xx - yy) > 1e-9) {
                    theta = 0.5 * std::atan2(2.0 * cxy, xx - yy);
                }

                // 2-sigma ellipse radii
                double a = 2.0 * std::sqrt(l1);
                double b = 2.0 * std::sqrt(l2);

                // Orientation quaternion around Z
                double half = theta * 0.5;
                cov_ellipse.pose.orientation.z = std::sin(half);
                cov_ellipse.pose.orientation.w = std::cos(half);

                // Points in local frame (no centroid offset added here)
                const int segments = 48;
                cov_ellipse.points.reserve(segments + 2);
                for (int i = 0; i <= segments; ++i) {
                    double ang = (2.0 * M_PI * i) / segments;
                    geometry_msgs::msg::Point p;
                    p.x = a * std::cos(ang);
                    p.y = b * std::sin(ang);
                    p.z = 0.0;
                    cov_ellipse.points.push_back(p);
                }
                ma.markers.push_back(std::move(cov_ellipse));
            }
            
            // Visualize predicted trajectory points
            if (t.position.size() > 1) {
                visualization_msgs::msg::Marker trajectory;
                trajectory.header.frame_id = frame_id;
                trajectory.header.stamp = stamp;
                trajectory.ns = "clusters_trajectory";
                trajectory.id = t.id;
                trajectory.type = visualization_msgs::msg::Marker::LINE_STRIP;
                trajectory.action = visualization_msgs::msg::Marker::ADD;
                trajectory.scale.x = 0.03;
                trajectory.color.r = 0.1f;
                trajectory.color.g = 0.8f;
                trajectory.color.b = 0.9f;
                trajectory.color.a = 0.8f;
                trajectory.lifetime = rclcpp::Duration(0, 7e8);
                trajectory.points.reserve(t.position.size());
                for (const auto &point : t.position) {
                    trajectory.points.push_back(point);
                }
                trajectory.frame_locked = true;
                ma.markers.push_back(std::move(trajectory));
            }

            // Increment ID for next marker
            ++id;
        }

        marker_pub_->publish(ma);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DynamicObstacleNode>();
    
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}
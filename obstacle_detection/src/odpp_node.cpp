#include "rclcpp/rclcpp.hpp"
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_costmap_2d/cost_values.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <chrono>
#include <memory>
#include <thread>
#include <vector>

#include "nav2_dynamic_msgs/msg/obstacle_array.hpp"
#include "nav2_dynamic_msgs/msg/obstacle.hpp"
#include "cluster_engine.hpp"
#include "types.hpp"

class DynamicObstacleNode : public rclcpp::Node
{
public:
    DynamicObstacleNode()
        : Node("dynamic_obstacle_node")
    {
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

        // Timer to process costmap (blob detection + tracking)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(200),
            std::bind(&DynamicObstacleNode::processCostmap, this));

        // Publishers
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("cluster_markers", 10);
        obstacle_pub_ = this->create_publisher<nav2_dynamic_msgs::msg::ObstacleArray>("obstacles_array", 10);

        RCLCPP_INFO(this->get_logger(), "DynamicObstacleNode started (clustering + RViz markers only).");
    }

private:
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
    std::unique_ptr<std::thread> costmap_thread_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<nav2_dynamic_msgs::msg::ObstacleArray>::SharedPtr obstacle_pub_;
    ClusterEngine cluster_engine_;

    // Main processing function: orchestrate clustering, tracking, publishing.
    void processCostmap()
    {
        auto costmap = costmap_ros_->getCostmap();
        if (!costmap)
            return;

        const unsigned char cost_threshold = nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;

        // 1) Extract clusters via ClusterEngine
        std::vector<BlobCluster> clusters = cluster_engine_.extract(costmap, cost_threshold);

        // 3) Publish messages
        publishClusterMarkers(clusters, costmap_ros_->getGlobalFrameID());
        publishObstacleArray(clusters, costmap_ros_->getGlobalFrameID()); // <-- add
    }

    void publishObstacleArray(const std::vector<BlobCluster> &clusters, const std::string &frame_id)
    {
        nav2_dynamic_msgs::msg::ObstacleArray msg;
        msg.header.stamp = now();
        msg.header.frame_id = frame_id;

        msg.obstacles.reserve(clusters.size());
        for (size_t i = 0; i < clusters.size(); ++i)
        {
            nav2_dynamic_msgs::msg::Obstacle ob;
            // Zero UUID for now (static/unknown)
            // ob.uuid.uuid.fill(0);

            ob.id = static_cast<int64_t>(i); // per-frame index (no tracking yet)
            // ob.score = 1.0f;

            // Positions: only current centroid as a single-element array
            ob.position.clear();
            ob.position.push_back(clusters[i].centroid);

            // Velocity/heading zero
            ob.velocity = geometry_msgs::msg::Vector3();
            ob.heading = geometry_msgs::msg::Vector3();

            // Polygon: convex hull as Point32
            geometry_msgs::msg::Polygon poly;
            poly.points.reserve(clusters[i].boundary.size());
            for (const auto &p : clusters[i].boundary)
            {
                geometry_msgs::msg::Point32 p32;
                p32.x = static_cast<float>(p.x);
                p32.y = static_cast<float>(p.y);
                p32.z = static_cast<float>(p.z);
                poly.points.push_back(p32);
            }
            ob.polygon = std::move(poly);

            // Covariances zero
            ob.position_covariance.fill(0.0);
            ob.velocity_covariance.fill(0.0);

            msg.obstacles.push_back(std::move(ob));
        }

        obstacle_pub_->publish(msg);
    }

    // Publish RViz markers for points, convex hull, and centroid per cluster
    void publishClusterMarkers(const std::vector<BlobCluster> &clusters, const std::string &frame_id)
    {
        visualization_msgs::msg::MarkerArray ma;
        rclcpp::Time stamp = now();

        // Clear previous markers
        {
            visualization_msgs::msg::Marker clear;
            clear.header.frame_id = frame_id;
            clear.header.stamp = stamp;
            clear.ns = "clusters";
            clear.id = 0;
            clear.action = visualization_msgs::msg::Marker::DELETEALL;
            ma.markers.push_back(clear);
        }

        int id = 1;

        for (const auto &c : clusters)
        {
            // Convex hull (closed)
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
            hull.frame_locked = true; // keep hull in place
            if (c.boundary.size() >= 3)
            {
                hull.points.push_back(c.boundary.front());
            }
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
            centroid.frame_locked = true; // keep centroid in place
            ma.markers.push_back(std::move(centroid));
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
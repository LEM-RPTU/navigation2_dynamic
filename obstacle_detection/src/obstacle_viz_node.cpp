#include "rclcpp/rclcpp.hpp"
#include "nav2_dynamic_interface/msg/obstacle_array.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <unordered_set>
#include <cmath>

class ObstacleVizNode : public rclcpp::Node
{
public:
    ObstacleVizNode()
        : Node("obstacle_viz_node")
    {
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("cluster_markers", 10);
        sub_ = this->create_subscription<nav2_dynamic_interface::msg::ObstacleArray>(
            "obstacles_array", 10,
            std::bind(&ObstacleVizNode::onObstacles, this, std::placeholders::_1));
        RCLCPP_INFO(get_logger(), "Obstacle viz node started");
    }

private:
    rclcpp::Subscription<nav2_dynamic_interface::msg::ObstacleArray>::SharedPtr sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    std::unordered_set<int64_t> active_ids_;

    void onObstacles(const nav2_dynamic_interface::msg::ObstacleArray::SharedPtr msg)
    {
        visualization_msgs::msg::MarkerArray ma;
        rclcpp::Time stamp = msg->header.stamp;
        const std::string frame_id = msg->header.frame_id;
        
        std::unordered_set<int64_t> current_ids;

        for (const auto &ob : msg->obstacles)
        {
            current_ids.insert(ob.id);
            
            if (ob.position.empty())
                continue;
            
            geometry_msgs::msg::Point centroid_pos = ob.position.front().pose.position;

            // Centroid marker
            visualization_msgs::msg::Marker centroid;
            centroid.header.frame_id = frame_id;
            centroid.header.stamp = stamp;
            centroid.ns = "clusters_centroid";
            centroid.id = ob.id;
            centroid.type = visualization_msgs::msg::Marker::SPHERE;
            centroid.action = visualization_msgs::msg::Marker::ADD;
            centroid.scale.x = 0.12;
            centroid.scale.y = 0.12;
            centroid.scale.z = 0.12;
            centroid.color.r = 0.2f;
            centroid.color.g = 1.0f;
            centroid.color.b = 0.2f;
            centroid.color.a = 0.95f;
            centroid.pose.position = centroid_pos;
            centroid.pose.orientation.w = 1.0;
            centroid.frame_locked = true;
            ma.markers.push_back(std::move(centroid));

            // Label (track ID)
            visualization_msgs::msg::Marker label;
            label.header.frame_id = frame_id;
            label.header.stamp = stamp;
            label.ns = "clusters_label";
            label.id = ob.id;
            label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            label.action = visualization_msgs::msg::Marker::ADD;
            label.scale.z = 0.25;
            label.color.r = 1.0f;
            label.color.g = 0.9f;
            label.color.b = 0.1f;
            label.color.a = 1.0f;
            label.lifetime = rclcpp::Duration(0, 7e8);
            label.pose.position = centroid_pos;
            label.pose.position.z += 0.3;
            label.pose.orientation.w = 1.0;
            label.text = std::to_string(ob.id);
            label.frame_locked = true;
            ma.markers.push_back(std::move(label));

            // Trajectory (extract points from PoseWithCovariance[])
            if (ob.position.size() > 1)
            {
                visualization_msgs::msg::Marker traj;
                traj.header.frame_id = frame_id;
                traj.header.stamp = stamp;
                traj.ns = "clusters_trajectory";
                traj.id = ob.id;
                traj.type = visualization_msgs::msg::Marker::LINE_STRIP;
                traj.action = visualization_msgs::msg::Marker::ADD;
                traj.scale.x = 0.03;
                traj.color.r = 0.1f;
                traj.color.g = 0.8f;
                traj.color.b = 0.9f;
                traj.color.a = 0.8f;
                traj.frame_locked = true;
                for (const auto &pose_cov : ob.position)
                {
                    traj.points.push_back(pose_cov.pose.position);
                }
                ma.markers.push_back(std::move(traj));
            }

            // Polygon hull
            if (!ob.polygon.points.empty())
            {
                visualization_msgs::msg::Marker hull;
                hull.header.frame_id = frame_id;
                hull.header.stamp = stamp;
                hull.ns = "clusters_hull";
                hull.id = ob.id;
                hull.type = visualization_msgs::msg::Marker::LINE_STRIP;
                hull.action = visualization_msgs::msg::Marker::ADD;
                hull.scale.x = 0.03;
                hull.color.r = 1.0f;
                hull.color.g = 0.3f;
                hull.color.b = 0.1f;
                hull.color.a = 0.9f;
                hull.frame_locked = true;
                for (const auto &p32 : ob.polygon.points)
                {
                    geometry_msgs::msg::Point p;
                    p.x = p32.x;
                    p.y = p32.y;
                    p.z = p32.z;
                    hull.points.push_back(p);
                }
                if (hull.points.size() >= 3)
                    hull.points.push_back(hull.points.front());
                ma.markers.push_back(std::move(hull));
            }

            // Covariance ellipse (extract from first PoseWithCovariance)
            const auto &cov = ob.position.front().covariance;
            if (cov[0] > 0.0 && cov[7] > 0.0)  // xx and yy
            {
                visualization_msgs::msg::Marker cov_ellipse;
                cov_ellipse.header.frame_id = frame_id;
                cov_ellipse.header.stamp = stamp;
                cov_ellipse.ns = "clusters_covariance";
                cov_ellipse.id = ob.id;
                cov_ellipse.type = visualization_msgs::msg::Marker::LINE_STRIP;
                cov_ellipse.action = visualization_msgs::msg::Marker::ADD;
                cov_ellipse.scale.x = 0.025;
                cov_ellipse.color.r = 0.95f;
                cov_ellipse.color.g = 0.95f;
                cov_ellipse.color.b = 0.10f;
                cov_ellipse.color.a = 0.85f;
                cov_ellipse.lifetime = rclcpp::Duration(0, 7e8);
                cov_ellipse.frame_locked = true;
                cov_ellipse.pose.position = centroid_pos;

                double xx = cov[0];
                double xy = cov[1];
                double yx = cov[6];
                double yy = cov[7];
                double cxy = 0.5 * (xy + yx);

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

                double theta = 0.0;
                if (std::fabs(cxy) > 1e-9 || std::fabs(xx - yy) > 1e-9)
                {
                    theta = 0.5 * std::atan2(2.0 * cxy, xx - yy);
                }

                double a = 2.0 * std::sqrt(l1);
                double b = 2.0 * std::sqrt(l2);

                double half = theta * 0.5;
                cov_ellipse.pose.orientation.z = std::sin(half);
                cov_ellipse.pose.orientation.w = std::cos(half);

                const int segments = 48;
                cov_ellipse.points.reserve(segments + 2);
                for (int i = 0; i <= segments; ++i)
                {
                    double ang = (2.0 * M_PI * i) / segments;
                    geometry_msgs::msg::Point p;
                    p.x = a * std::cos(ang);
                    p.y = b * std::sin(ang);
                    p.z = 0.0;
                    cov_ellipse.points.push_back(p);
                }
                ma.markers.push_back(std::move(cov_ellipse));
            }

            // Velocity arrow
            if (!ob.velocity.empty())
            {
                const auto &vel = ob.velocity.front().twist.linear;
                double speed = std::sqrt(vel.x * vel.x + vel.y * vel.y);
                
                if (speed > 0.05)  // Only show if moving (threshold 5 cm/s)
                {
                    visualization_msgs::msg::Marker vel_arrow;
                    vel_arrow.header.frame_id = frame_id;
                    vel_arrow.header.stamp = stamp;
                    vel_arrow.ns = "clusters_velocity";
                    vel_arrow.id = ob.id;
                    vel_arrow.type = visualization_msgs::msg::Marker::ARROW;
                    vel_arrow.action = visualization_msgs::msg::Marker::ADD;
                    
                    // Arrow from centroid to centroid + velocity vector
                    geometry_msgs::msg::Point start = centroid_pos;
                    geometry_msgs::msg::Point end;
                    end.x = centroid_pos.x + vel.x;
                    end.y = centroid_pos.y + vel.y;
                    end.z = centroid_pos.z;
                    
                    vel_arrow.points.push_back(start);
                    vel_arrow.points.push_back(end);
                    
                    // Arrow appearance
                    vel_arrow.scale.x = 0.08;  // Shaft diameter
                    vel_arrow.scale.y = 0.15;  // Head diameter
                    vel_arrow.scale.z = 0.0;   // Head length (auto)
                    
                    // Color by speed (green=slow, red=fast)
                    float speed_norm = std::min(speed / 2.0, 1.0);  // Normalize to [0,1] assuming max 2 m/s
                    vel_arrow.color.r = speed_norm;
                    vel_arrow.color.g = 1.0f - speed_norm;
                    vel_arrow.color.b = 0.2f;
                    vel_arrow.color.a = 0.9f;
                    
                    vel_arrow.frame_locked = true;
                    ma.markers.push_back(std::move(vel_arrow));
                }
            }
        }

        // Delete markers for retired IDs
        for (int64_t retired_id : active_ids_)
        {
            if (current_ids.find(retired_id) == current_ids.end())
            {
                std::vector<std::string> namespaces = {
                    "clusters_centroid", "clusters_label", "clusters_trajectory",
                    "clusters_hull", "clusters_covariance", "clusters_velocity"
                };
                for (const auto &ns : namespaces)
                {
                    visualization_msgs::msg::Marker delete_marker;
                    delete_marker.header = msg->header;
                    delete_marker.ns = ns;
                    delete_marker.id = static_cast<int>(retired_id);
                    delete_marker.action = visualization_msgs::msg::Marker::DELETE;
                    ma.markers.push_back(delete_marker);
                }
            }
        }

        active_ids_ = current_ids;
        marker_pub_->publish(ma);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObstacleVizNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
#include "rclcpp/rclcpp.hpp"
#include "nav2_dynamic_interface/msg/obstacle_array.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/point.hpp"

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

    void onObstacles(const nav2_dynamic_interface::msg::ObstacleArray::SharedPtr msg)
    {
        visualization_msgs::msg::MarkerArray ma;
        rclcpp::Time stamp = msg->header.stamp;
        const std::string frame_id = msg->header.frame_id;

        // Delete previous markers
        visualization_msgs::msg::Marker clear;
        clear.header.frame_id = frame_id;
        clear.header.stamp = stamp;
        clear.ns = "clusters";
        clear.id = 0;
        clear.action = visualization_msgs::msg::Marker::DELETEALL;
        ma.markers.push_back(clear);

        for (const auto &ob : msg->obstacles)
        {
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
            if (!ob.position.empty())
                centroid.pose.position = ob.position.front();
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
            if (!ob.position.empty())
                label.pose.position = ob.position.front();
            label.pose.position.z += 0.3;
            label.pose.orientation.w = 1.0;
            label.text = std::to_string(ob.id);
            label.frame_locked = true;
            ma.markers.push_back(std::move(label));

            // Trajectory
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
                traj.points = ob.position;
                ma.markers.push_back(std::move(traj));
            }

            // Optional: polygon hull visualization (if polygon present)
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
                // convert Point32 -> Point
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

            // (Add covariance ellipse or labels as needed — keep this node focused and minimal)
            // Add covariance ellipse visualization
            if (ob.position_covariance[0] > 0.0 && ob.position_covariance[3] > 0.0)
            {
                visualization_msgs::msg::Marker cov_ellipse;
                cov_ellipse.header.frame_id = frame_id;
                cov_ellipse.header.stamp = stamp;
                cov_ellipse.ns = "clusters_covariance";
                cov_ellipse.id = ob.id;
                cov_ellipse.type = visualization_msgs::msg::Marker::LINE_STRIP;
                cov_ellipse.action = visualization_msgs::msg::Marker::ADD;
                cov_ellipse.scale.x = 0.025; // line width
                cov_ellipse.color.r = 0.95f;
                cov_ellipse.color.g = 0.95f;
                cov_ellipse.color.b = 0.10f;
                cov_ellipse.color.a = 0.85f;
                cov_ellipse.lifetime = rclcpp::Duration(0, 7e8);
                cov_ellipse.frame_locked = true;

                // Place ellipse at centroid
                cov_ellipse.pose.position = ob.position.front();

                // Cov matrix (row-major 2x2)
                double xx = ob.position_covariance[0];
                double xy = ob.position_covariance[1];
                double yx = ob.position_covariance[2];
                double yy = ob.position_covariance[3];

                // Force symmetry (defensive)
                double cxy = 0.5 * (xy + yx);

                // Eigenvalues
                double trace = xx + yy;
                double det = xx * yy - cxy * cxy;
                if (det < 0.0)
                    det = 0.0;
                double disc = trace * trace - 4.0 * det;
                if (disc < 0.0)
                    disc = 0.0;
                double sqrt_disc = std::sqrt(disc);
                double l1 = 0.5 * (trace + sqrt_disc);
                double l2 = 0.5 * (trace - sqrt_disc);
                if (l1 < 1e-9)
                    l1 = 1e-9;
                if (l2 < 1e-9)
                    l2 = 1e-9;

                // Principal axis angle (atan2 for symmetric 2x2)
                double theta = 0.0;
                if (std::fabs(cxy) > 1e-9 || std::fabs(xx - yy) > 1e-9)
                {
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

            marker_pub_->publish(ma);
        }
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
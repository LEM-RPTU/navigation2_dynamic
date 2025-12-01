#include <cmath>
#include <string>
#include <unordered_set>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "nav2_dynamic_interface/msg/obstacle_sequence_array.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/point.hpp"

/**
 * @class ObstacleVizNode
 * @brief Visualization node for dynamic obstacles
 * 
 * Subscribes to Obstacle Sequence and publishes RViz markers:
 * - Centroid spheres (current position)
 * - Track ID labels
 * - Predicted trajectories (line strips)
 * - Convex hull boundaries
 * - Covariance ellipses (for all predicted positions)
 * - Velocity arrows (color-coded by speed)
 */
class ObstacleVizNode : public rclcpp::Node
{
public:
  ObstacleVizNode() : Node("obstacle_viz_node"){
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "obstacle_markers", 10);

    sub_ = this->create_subscription<nav2_dynamic_interface::msg::ObstacleSequenceArray>(
      "obstacles_sequences", 10,
      std::bind(&ObstacleVizNode::onObstacles, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Obstacle visualization node started");
  }

private:
  rclcpp::Subscription<nav2_dynamic_interface::msg::ObstacleSequenceArray>::SharedPtr sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  std::unordered_set<int64_t> active_ids_;

  void onObstacles(const nav2_dynamic_interface::msg::ObstacleSequenceArray::SharedPtr msg){
    visualization_msgs::msg::MarkerArray ma;
    
    
    rclcpp::Time stamp;
    std::string frame_id;
    std::unordered_set<int64_t> current_ids;

    for (const nav2_dynamic_interface::msg::ObstacleSequence &seq_ref : msg->obstacle_sequences) {
      nav2_dynamic_interface::msg::ObstacleSequence sequence = seq_ref;
      current_ids.insert(sequence.id);
      stamp = sequence.header.stamp;
      frame_id = sequence.header.frame_id;
      if (sequence.positions.empty()) {
        continue;
      }

      geometry_msgs::msg::Point centroid_pos = sequence.positions.front().pose.position;

      // Centroid marker (current position - green sphere)
      visualization_msgs::msg::Marker centroid;
      centroid.header.frame_id = frame_id;
      centroid.header.stamp = stamp;
      centroid.ns = "clusters_centroid";
      centroid.id = sequence.id;
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

      // Label (track ID as text)
      visualization_msgs::msg::Marker label;
      label.header.frame_id = frame_id;
      label.header.stamp = stamp;
      label.ns = "clusters_label";
      label.id = sequence.id;
      label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      label.action = visualization_msgs::msg::Marker::ADD;
      label.scale.z = 0.25;
      label.color.r = 1.0f;
      label.color.g = 0.9f;
      label.color.b = 0.1f;
      label.color.a = 1.0f;
      label.lifetime = rclcpp::Duration(0, 700000000);  // 700ms
      label.pose.position = centroid_pos;
      label.pose.position.z += 0.3;
      label.pose.orientation.w = 1.0;
      label.text = std::to_string(sequence.id);
      label.frame_locked = true;
      ma.markers.push_back(std::move(label));

      // Trajectory (predicted future path)
      if (sequence.positions.size() > 1) {
        visualization_msgs::msg::Marker traj;
        traj.header.frame_id = frame_id;
        traj.header.stamp = stamp;
        traj.ns = "clusters_trajectory";
        traj.id = sequence.id;
        traj.type = visualization_msgs::msg::Marker::LINE_STRIP;
        traj.action = visualization_msgs::msg::Marker::ADD;
        traj.scale.x = 0.03;
        traj.color.r = 0.1f;
        traj.color.g = 0.8f;
        traj.color.b = 0.9f;
        traj.color.a = 0.8f;
        traj.frame_locked = true;
        for (const geometry_msgs::msg::PoseWithCovariance &pose_cov : sequence.positions) {
          traj.points.push_back(pose_cov.pose.position);
        }
        ma.markers.push_back(std::move(traj));
      }

      // Polygon hull (convex boundary - only for current position)
      bool no_polygons = sequence.polygons.empty();
      if(no_polygons){
        RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 500, "Obstacle ID %ld has no polygon data for hull visualization.", sequence.id);
      }else{
        if(!sequence.polygons.front().points.empty()){
          visualization_msgs::msg::Marker hull;
          hull.header.frame_id = frame_id;
          hull.header.stamp = stamp;
          hull.ns = "clusters_hull";
          hull.id = sequence.id;
          hull.type = visualization_msgs::msg::Marker::LINE_STRIP;
          hull.action = visualization_msgs::msg::Marker::ADD;
          hull.scale.x = 0.03;
          hull.color.r = 1.0f;
          hull.color.g = 0.3f;
          hull.color.b = 0.1f;
          hull.color.a = 0.9f;
          hull.frame_locked = true;
          for (const geometry_msgs::msg::Point32 &polygon_point : sequence.polygons.front().points) {
            geometry_msgs::msg::Point point;
            point.x = polygon_point.x;
            point.y = polygon_point.y;
            point.z = polygon_point.z;
            hull.points.push_back(point);
          }
          // Close the loop
          if (hull.points.size() >= 3) {
            hull.points.push_back(hull.points.front());
          }
          ma.markers.push_back(std::move(hull));
        }
      }

      // Covariance ellipses for ALL predicted positions
      int32_t position_index = 0;
      for (const geometry_msgs::msg::PoseWithCovariance &position: sequence.positions) {
        const geometry_msgs::msg::Pose &pose = position.pose;
        const std::array<double, 36UL> &covariance = position.covariance;

        // Check if covariance is valid
        if (covariance[0] <= 0.0 || covariance[7] <= 0.0) {
          continue;  // Skip invalid covariance
        }

        visualization_msgs::msg::Marker cov_ellipse;
        cov_ellipse.header.frame_id = frame_id;
        cov_ellipse.header.stamp = stamp;
        cov_ellipse.ns = "clusters_covariance";
        // Unique ID: encode both obstacle ID and position index
        cov_ellipse.id = static_cast<int>(sequence.id * 1000 + position_index);
        cov_ellipse.type = visualization_msgs::msg::Marker::LINE_STRIP;
        cov_ellipse.action = visualization_msgs::msg::Marker::ADD;
        cov_ellipse.scale.x = 0.02;

        // Color fading: current (yellow) → future (faded yellow)
        float fade = 1.0f - (static_cast<float>(position_index) / sequence.positions.size()) * 0.5f;
        cov_ellipse.color.r = 0.95f * fade;
        cov_ellipse.color.g = 0.95f * fade;
        cov_ellipse.color.b = 0.10f * fade;
        cov_ellipse.color.a = 0.7f * fade;

        cov_ellipse.lifetime = rclcpp::Duration(0, 700000000);
        cov_ellipse.frame_locked = true;
        cov_ellipse.pose.position = pose.position;

        // Extract 2x2 covariance submatrix
        double xx = covariance[0];
        double xy = covariance[1];
        double yx = covariance[6];
        double yy = covariance[7];
        double cxy = 0.5 * (xy + yx);  // Symmetrize

        // Compute eigenvalues
        double trace = xx + yy;
        double det = xx * yy - cxy * cxy;
        if (det < 0.0) {det = 0.0;}
        double disc = trace * trace - 4.0 * det;
        if (disc < 0.0) {disc = 0.0;}
        double sqrt_disc = std::sqrt(disc);
        double l1 = 0.5 * (trace + sqrt_disc);
        double l2 = 0.5 * (trace - sqrt_disc);
        if (l1 < 1e-9) {l1 = 1e-9;}
        if (l2 < 1e-9) {l2 = 1e-9;}

        // Compute rotation angle
        double theta = 0.0;
        if (std::fabs(cxy) > 1e-9 || std::fabs(xx - yy) > 1e-9) {
          theta = 0.5 * std::atan2(2.0 * cxy, xx - yy);
        }

        // Ellipse axes (2-sigma confidence interval)
        double a = 2.0 * std::sqrt(l1);
        double b = 2.0 * std::sqrt(l2);

        // Set orientation
        double half = theta * 0.5;
        cov_ellipse.pose.orientation.z = std::sin(half);
        cov_ellipse.pose.orientation.w = std::cos(half);

        // Generate ellipse points
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
        position_index++;
      }

      // Velocity arrows for ALL predicted velocities
      int32_t velocity_index = 0;
      for (const geometry_msgs::msg::TwistWithCovariance &twist : sequence.velocities) {
        const geometry_msgs::msg::Vector3 &velocity = twist.twist.linear;
        double speed = std::sqrt(velocity.x * velocity.x + velocity.y * velocity.y);

        if (speed < 0.05) {  // Skip near-stationary
          continue;
        }

        // Use corresponding position if available, otherwise use current
        geometry_msgs::msg::Point start_pos;
        if(velocity_index < sequence.positions.size()){
          start_pos = sequence.positions[velocity_index].pose.position;
        }else{
          start_pos = centroid_pos;
        }

        visualization_msgs::msg::Marker vel_arrow;
        vel_arrow.header.frame_id = frame_id;
        vel_arrow.header.stamp = stamp;
        vel_arrow.ns = "clusters_velocity";
        // Unique ID: encode obstacle ID and velocity index
        vel_arrow.id = static_cast<int>(sequence.id * 1000 + velocity_index);
        vel_arrow.type = visualization_msgs::msg::Marker::ARROW;
        vel_arrow.action = visualization_msgs::msg::Marker::ADD;

        // Arrow from position along velocity vector
        geometry_msgs::msg::Point start = start_pos;
        geometry_msgs::msg::Point end;
        end.x = start_pos.x + velocity.x * 0.5;  // Scale for visibility (0.5s lookahead)
        end.y = start_pos.y + velocity.y * 0.5;
        end.z = start_pos.z;

        vel_arrow.points.push_back(start);
        vel_arrow.points.push_back(end);

        // Arrow styling (smaller for future predictions)
        float scale_factor = 1.0f - (static_cast<float>(velocity_index) / sequence.velocities.size()) * 0.3f;
        vel_arrow.scale.x = 0.06 * scale_factor;  // Shaft diameter
        vel_arrow.scale.y = 0.12 * scale_factor;  // Head diameter
        vel_arrow.scale.z = 0.0;                   // Head length (auto)

        // Color interpolation (green → red based on speed)
        float speed_norm = std::min(static_cast<float>(speed / 2.0), 1.0f);
        float fade = 1.0f - (static_cast<float>(velocity_index) / sequence.velocities.size()) * 0.4f;
        vel_arrow.color.r = speed_norm * fade;
        vel_arrow.color.g = (1.0f - speed_norm) * fade;
        vel_arrow.color.b = 0.2f * fade;
        vel_arrow.color.a = 0.8f * fade;

        vel_arrow.lifetime = rclcpp::Duration(0, 700000000);
        vel_arrow.frame_locked = true;
        ma.markers.push_back(std::move(vel_arrow));
        velocity_index++;
      }

      // Predicted position markers (small spheres for future positions)
      position_index = 0;
      for (const geometry_msgs::msg::PoseWithCovariance &position : sequence.positions) {
        visualization_msgs::msg::Marker pred_sphere;
        pred_sphere.header.frame_id = frame_id;
        pred_sphere.header.stamp = stamp;
        pred_sphere.ns = "clusters_predictions";
        pred_sphere.id = static_cast<int>(sequence.id * 1000 + position_index);
        pred_sphere.type = visualization_msgs::msg::Marker::SPHERE;
        pred_sphere.action = visualization_msgs::msg::Marker::ADD;

        // Size decreases with distance into future
        float scale_factor = 1.0f - (static_cast<float>(position_index) / sequence.positions.size()) * 0.5f;
        pred_sphere.scale.x = 0.08 * scale_factor;
        pred_sphere.scale.y = 0.08 * scale_factor;
        pred_sphere.scale.z = 0.08 * scale_factor;

        // Color fades from cyan to dark blue
        float fade = 1.0f - (static_cast<float>(position_index) / sequence.positions.size()) * 0.6f;
        pred_sphere.color.r = 0.1f * fade;
        pred_sphere.color.g = 0.6f * fade;
        pred_sphere.color.b = 0.9f * fade;
        pred_sphere.color.a = 0.7f * fade;

        pred_sphere.pose.position = position.pose.position;
        pred_sphere.pose.orientation.w = 1.0;
        pred_sphere.lifetime = rclcpp::Duration(0, 700000000);
        pred_sphere.frame_locked = true;
        ma.markers.push_back(std::move(pred_sphere));
        position_index++;
      }
    }

    // Delete markers for retired tracks
    for (int64_t retired_id : active_ids_) {
      if (current_ids.find(retired_id) == current_ids.end()) {
        // Delete all marker types for this ID
        std::vector<std::string> namespaces = {
          "clusters_centroid", "clusters_label", "clusters_trajectory",
          "clusters_hull", "clusters_covariance", "clusters_velocity",
          "clusters_predictions"
        };
        
        for (const auto & ns : namespaces) {
          // Delete base marker
          visualization_msgs::msg::Marker delete_marker;
          delete_marker.header = msg->header;
          delete_marker.ns = ns;
          delete_marker.id = static_cast<int>(retired_id);
          delete_marker.action = visualization_msgs::msg::Marker::DELETE;
          ma.markers.push_back(delete_marker);

          // Delete indexed markers (covariance, velocity, predictions)
          if (ns == "clusters_covariance" || ns == "clusters_velocity" || ns == "clusters_predictions") {
            for (int idx = 0; idx < 100; ++idx) {  // Assume max 100 predictions
              visualization_msgs::msg::Marker delete_indexed;
              delete_indexed.header = msg->header;
              delete_indexed.ns = ns;
              delete_indexed.id = static_cast<int>(retired_id * 1000 + idx);
              delete_indexed.action = visualization_msgs::msg::Marker::DELETE;
              ma.markers.push_back(delete_indexed);
            }
          }
        }
      }
    }

    active_ids_ = current_ids;
    marker_pub_->publish(ma);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ObstacleVizNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
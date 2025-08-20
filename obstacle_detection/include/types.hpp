#pragma once

#include <vector>
#include <array>
#include <cstdint>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>
#include <rclcpp/time.hpp>
#include "rclcpp/rclcpp.hpp"

// Output of the ClusterEngine
struct BlobCluster
{
  // Raw blob points in world coordinates (meters).
  std::vector<geometry_msgs::msg::Point> points;

  // Centroid of the blob points (world coords).
  geometry_msgs::msg::Point centroid;

  // Convex hull (counter-clockwise) of the blob points (world coords).
  std::vector<geometry_msgs::msg::Point> boundary;
};

// Per-cluster assignment after tracking
struct ClusterAssignment
{
  size_t cluster_index;                 // index into current clusters
  int64_t id;                           // persistent track ID
  double cost;                          // association distance
  bool new_track;                       // true if newly created track this tick
};

// Shared internal track data (tracker <-> predictor),
// aligned with nav2_dynamic_msgs/Obstacle fields.
struct ObstacleTrack
{
  // Message-aligned fields
  unique_identifier_msgs::msg::UUID uuid{}; // 16-byte UUID
  int64_t id{0};                            // integer ID
  float score{1.0f};                        // detection confidence

  // position[0] = current centroid, position[1..N] = predictions
  std::vector<geometry_msgs::msg::Point> position;

  geometry_msgs::msg::Vector3 velocity{}; // meters/second
  geometry_msgs::msg::Vector3 heading{};  // unit direction (x,y), z=0
  geometry_msgs::msg::Polygon polygon{};  // convex hull

  // 2x2 covariance (row-major): [xx, xy, yx, yy]
  std::array<double, 4> position_covariance{{0.0, 0.0, 0.0, 0.0}};
  std::array<double, 4> velocity_covariance{{0.0, 0.0, 0.0, 0.0}};

  // Internal-only fields (not in the .msg)
  std::vector<geometry_msgs::msg::Point> history; // past centroids, most-recent at back
  bool is_dynamic{false};
};

struct TrackerPredictionFrame
{
  rclcpp::Time stamp;
  std::vector<ObstacleTrack> tracks;          // All active obstacle tracks
  std::vector<ClusterAssignment> assignments; // Cluster-to-track assignments
  std::vector<int64_t> retired_tracks;        // IDs of tracks removed this frame
};
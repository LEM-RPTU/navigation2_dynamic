#pragma once

#include <vector>
#include <cstdint>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>

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
  size_t cluster_index;                   // index into current clusters
  int64_t id;                             // persistent track ID
  double cost;                            // association distance
  bool new_track;                         // true if newly created track this tick
  geometry_msgs::msg::Vector3 velocity;   // estimated velocity of the track (dx, dy per tick)
};

// Result of a tracker update
struct TrackingResult
{
  std::vector<ClusterAssignment> assignments; // one entry for each matched or birthed cluster
  std::vector<size_t> unmatched_clusters;     // clusters that failed gate (shouldn’t occur since we birth)
  std::vector<int64_t> retired_tracks;        // tracks dropped due to aging
};


#pragma once

#include <vector>
#include <cstdint>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>
#include <builtin_interfaces/msg/duration.hpp>

/**
 * @struct BlobCluster
 * @brief Output of ClusterEngine: connected component blob from costmap
 */
struct BlobCluster
{
  std::vector<geometry_msgs::msg::Point> points;    ///< All blob points (world coords)
  geometry_msgs::msg::Point centroid;               ///< Geometric center (Shoelace)
  std::vector<geometry_msgs::msg::Point> boundary;  ///< Convex hull vertices (CCW)
};

/**
 * @struct ObstacleTrack
 * @brief Persistent multi-hypothesis track state
 * 
 * Unified track representation used throughout detection → tracking → prediction pipeline.
 */
struct ObstacleTrack
{
  unique_identifier_msgs::msg::UUID uuid{};  ///< Unique identifier (for static obstacles)
  int64_t id{0};                             ///< Track ID (assigned by tracker)

  // Current detection (this cycle)
  geometry_msgs::msg::Point current_position;  ///< Latest matched detection centroid
  geometry_msgs::msg::Polygon polygon;         ///< Convex hull boundary

  // Predicted position for NEXT cycle (from predictor)
  geometry_msgs::msg::Point predicted_next_position;  ///< Expected position at t+1

  // Tracking state
  int missed{0};  ///< Consecutive missed detection counter (for retirement)
};
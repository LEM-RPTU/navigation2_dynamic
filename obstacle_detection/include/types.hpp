#pragma once

#include <vector>
#include <cstdint>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>
#include <builtin_interfaces/msg/duration.hpp>

// Output of ClusterEngine
struct BlobCluster
{
  std::vector<geometry_msgs::msg::Point> points;
  geometry_msgs::msg::Point centroid;
  std::vector<geometry_msgs::msg::Point> boundary;
};

// Single unified track struct (used everywhere)
struct ObstacleTrack
{
  unique_identifier_msgs::msg::UUID uuid{};
  int64_t id{0};

  // Current detection (this cycle)
  geometry_msgs::msg::Point current_position;
  geometry_msgs::msg::Polygon polygon;

  // Predicted position for NEXT cycle (from predictor)
  geometry_msgs::msg::Point predicted_next_position;

  // Tracking state
  int missed{0};
};
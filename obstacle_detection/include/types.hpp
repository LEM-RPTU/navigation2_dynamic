#pragma once

#include <vector>
#include <geometry_msgs/msg/point.hpp>

struct BlobCluster
{
  // Raw blob points in world coordinates (meters).
  std::vector<geometry_msgs::msg::Point> points;

  // Centroid of the blob points (world coords).
  geometry_msgs::msg::Point centroid;

  // Convex hull (counter-clockwise) of the blob points (world coords).
  std::vector<geometry_msgs::msg::Point> boundary;
};
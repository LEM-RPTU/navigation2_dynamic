#pragma once

#include <vector>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <geometry_msgs/msg/point.hpp>
#include "types.hpp"

class ClusterEngine
{
public:
  ClusterEngine() = default;

  // Extract connected components (8-connectivity) from a costmap at/above cost_threshold.
  // Returns clusters with points mapped to world coordinates and computed centroid + convex hull.
  std::vector<BlobCluster> extract(nav2_costmap_2d::Costmap2D* costmap,
                                   unsigned char cost_threshold) const;

private:
  // Compute centroid of a polygon using Shoelace formula
  static geometry_msgs::msg::Point computePolygonCentroid(const std::vector<geometry_msgs::msg::Point>& hull);
  
  // Compute convex hull (Andrew's monotone chain)
  static std::vector<geometry_msgs::msg::Point> computeConvexHull(std::vector<geometry_msgs::msg::Point> pts);
  
  // Cross product for convex hull algorithm
  static double cross(const geometry_msgs::msg::Point& O,
                     const geometry_msgs::msg::Point& A,
                     const geometry_msgs::msg::Point& B);
};
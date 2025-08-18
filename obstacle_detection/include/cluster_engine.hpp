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
  static geometry_msgs::msg::Point computeCentroid(const std::vector<geometry_msgs::msg::Point>& pts);

  // Monotone chain convex hull (O(n log n)), returns hull in CCW order without duplicate last point.
  static std::vector<geometry_msgs::msg::Point> computeConvexHull(std::vector<geometry_msgs::msg::Point> pts);

  // Cross product (OA x OB)
  static double cross(const geometry_msgs::msg::Point& O,
                      const geometry_msgs::msg::Point& A,
                      const geometry_msgs::msg::Point& B);
};
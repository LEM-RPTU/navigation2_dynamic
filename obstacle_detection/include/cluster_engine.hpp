#pragma once

#include <vector>

#include <nav2_costmap_2d/costmap_2d.hpp>
#include <geometry_msgs/msg/point.hpp>

#include "types.hpp"

/**
 * @class ClusterEngine
 * @brief Extract connected component blobs from costmap using flood fill
 * 
 * Identifies high-cost regions in a costmap, computes convex hulls, and
 * calculates geometric centroids using the Shoelace formula.
 */
class ClusterEngine
{
public:
  ClusterEngine() = default;

  /**
   * @brief Extract blob clusters from costmap
   * @param costmap Pointer to Nav2 costmap
   * @param cost_threshold Minimum cost value to include in blob (typically INSCRIBED_INFLATED_OBSTACLE)
   * @return Vector of clusters with points, centroids, and convex hull boundaries
   * 
   * Uses 8-connected flood fill to identify blobs, then computes:
   * - All blob points (world coordinates)
   * - Convex hull boundary (Andrew's monotone chain algorithm)
   * - Geometric centroid (Shoelace formula on hull vertices)
   */
  std::vector<BlobCluster> extract(
    nav2_costmap_2d::Costmap2D * costmap,
    unsigned char cost_threshold) const;

private:
  /**
   * @brief Compute polygon centroid using Shoelace formula
   * @param hull Convex hull vertices (CCW order)
   * @return Geometric centroid point
   * 
   * Handles degenerate cases (< 3 vertices, zero area) by fallback to average.
   */
  static geometry_msgs::msg::Point computePolygonCentroid(
    const std::vector<geometry_msgs::msg::Point> & hull);

  /**
   * @brief Compute convex hull using Andrew's monotone chain algorithm
   * @param pts Unordered set of 2D points
   * @return Hull vertices in CCW order
   */
  static std::vector<geometry_msgs::msg::Point> computeConvexHull(
    std::vector<geometry_msgs::msg::Point> pts);

  /**
   * @brief 2D cross product for convex hull orientation test
   * @param O Origin point
   * @param A First point
   * @param B Second point
   * @return Cross product (OA × OB)_z component
   */
  static double cross(
    const geometry_msgs::msg::Point & O,
    const geometry_msgs::msg::Point & A,
    const geometry_msgs::msg::Point & B);
};
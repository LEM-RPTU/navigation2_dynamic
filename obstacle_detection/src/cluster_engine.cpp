#include "cluster_engine.hpp"

#include <algorithm>
#include <utility>
#include <cmath>

geometry_msgs::msg::Point ClusterEngine::computePolygonCentroid(
  const std::vector<geometry_msgs::msg::Point> & hull)
{
  geometry_msgs::msg::Point centroid;
  centroid.x = 0.0;
  centroid.y = 0.0;
  centroid.z = 0.0;

  if (hull.size() < 3) {
    // Degenerate case: use simple average
    if (hull.empty()) {
      return centroid;
    }
    for (const auto & p : hull) {
      centroid.x += p.x;
      centroid.y += p.y;
    }
    centroid.x /= static_cast<double>(hull.size());
    centroid.y /= static_cast<double>(hull.size());
    return centroid;
  }

  // Shoelace formula for polygon centroid
  double A = 0.0;   // Signed area
  double Cx = 0.0;  // Centroid x accumulator
  double Cy = 0.0;  // Centroid y accumulator

  const size_t n = hull.size();
  for (size_t i = 0; i < n; ++i) {
    size_t j = (i + 1) % n;  // Next vertex (wraps around)

    double cross = hull[i].x * hull[j].y - hull[j].x * hull[i].y;
    A += cross;
    Cx += (hull[i].x + hull[j].x) * cross;
    Cy += (hull[i].y + hull[j].y) * cross;
  }

  A *= 0.5;  // Signed area

  if (std::abs(A) < 1e-9) {
    // Near-zero area (collinear points): fallback to average
    for (const auto & p : hull) {
      centroid.x += p.x;
      centroid.y += p.y;
    }
    centroid.x /= static_cast<double>(hull.size());
    centroid.y /= static_cast<double>(hull.size());
    return centroid;
  }

  centroid.x = Cx / (6.0 * A);
  centroid.y = Cy / (6.0 * A);
  return centroid;
}

double ClusterEngine::cross(
  const geometry_msgs::msg::Point & O,
  const geometry_msgs::msg::Point & A,
  const geometry_msgs::msg::Point & B)
{
  return (A.x - O.x) * (B.y - O.y) - (A.y - O.y) * (B.x - O.x);
}

std::vector<geometry_msgs::msg::Point> ClusterEngine::computeConvexHull(
  std::vector<geometry_msgs::msg::Point> pts)
{
  std::vector<geometry_msgs::msg::Point> H;
  if (pts.size() <= 1) {
    return pts;
  }

  // Sort points lexicographically (x, then y)
  std::sort(
    pts.begin(), pts.end(), [](const auto & a, const auto & b) {
      if (a.x < b.x) {return true;}
      if (a.x > b.x) {return false;}
      return a.y < b.y;
    });

  // Build lower hull
  for (const auto & p : pts) {
    while (H.size() >= 2 && cross(H[H.size() - 2], H.back(), p) <= 0.0) {
      H.pop_back();
    }
    H.push_back(p);
  }

  // Build upper hull
  size_t lower_size = H.size();
  for (int i = static_cast<int>(pts.size()) - 2; i >= 0; --i) {
    const auto & p = pts[static_cast<size_t>(i)];
    while (H.size() > lower_size && cross(H[H.size() - 2], H.back(), p) <= 0.0) {
      H.pop_back();
    }
    H.push_back(p);
  }

  // Remove duplicate last point (same as first)
  if (!H.empty()) {
    H.pop_back();
  }
  return H;
}

std::vector<BlobCluster> ClusterEngine::extract(
  nav2_costmap_2d::Costmap2D * costmap,
  unsigned char cost_threshold) const
{
  std::vector<BlobCluster> clusters;
  if (!costmap) {
    return clusters;
  }

  const unsigned int size_x = costmap->getSizeInCellsX();
  const unsigned int size_y = costmap->getSizeInCellsY();

  // Visited flags for flood fill
  std::vector<std::vector<bool>> visited(size_x, std::vector<bool>(size_y, false));
  using Cell = std::pair<unsigned int, unsigned int>;

  // Flood fill lambda (8-connectivity)
  auto floodFill = [&](
    unsigned int sx, unsigned int sy,
    std::vector<geometry_msgs::msg::Point> & out_points) {
      std::vector<Cell> stack;
      stack.emplace_back(sx, sy);

      while (!stack.empty()) {
        auto [cx, cy] = stack.back();
        stack.pop_back();

        if (visited[cx][cy]) {
          continue;
        }
        visited[cx][cy] = true;

        if (costmap->getCost(cx, cy) >= cost_threshold) {
          // Convert cell to world coordinates
          double wx, wy;
          costmap->mapToWorld(cx, cy, wx, wy);
          geometry_msgs::msg::Point p;
          p.x = wx;
          p.y = wy;
          p.z = 0.0;
          out_points.push_back(p);

          // Explore 8-connected neighbors
          for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
              if (dx == 0 && dy == 0) {
                continue;
              }
              int nx = static_cast<int>(cx) + dx;
              int ny = static_cast<int>(cy) + dy;
              if (nx >= 0 && ny >= 0 &&
                nx < static_cast<int>(size_x) &&
                ny < static_cast<int>(size_y) &&
                !visited[static_cast<size_t>(nx)][static_cast<size_t>(ny)])
              {
                stack.emplace_back(
                  static_cast<unsigned int>(nx),
                  static_cast<unsigned int>(ny));
              }
            }
          }
        }
      }
    };

  // Scan costmap for high-cost regions
  for (unsigned int x = 0; x < size_x; ++x) {
    for (unsigned int y = 0; y < size_y; ++y) {
      if (!visited[x][y] && costmap->getCost(x, y) >= cost_threshold) {
        std::vector<geometry_msgs::msg::Point> pts;
        floodFill(x, y, pts);
        if (!pts.empty()) {
          BlobCluster cluster;
          cluster.points = std::move(pts);

          // Compute convex hull first (for geometric accuracy)
          cluster.boundary = computeConvexHull(cluster.points);

          // Compute centroid from convex hull (Shoelace formula)
          cluster.centroid = computePolygonCentroid(cluster.boundary);

          clusters.push_back(std::move(cluster));
        }
      }
    }
  }

  return clusters;
}
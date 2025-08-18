#include "cluster_engine.hpp"

#include <algorithm>
#include <utility>

geometry_msgs::msg::Point ClusterEngine::computeCentroid(const std::vector<geometry_msgs::msg::Point>& pts)
{
  geometry_msgs::msg::Point c;
  c.x = 0.0; c.y = 0.0; c.z = 0.0;
  if (pts.empty()) return c;
  for (const auto& p : pts) {
    c.x += p.x;
    c.y += p.y;
  }
  c.x /= static_cast<double>(pts.size());
  c.y /= static_cast<double>(pts.size());
  return c;
}

double ClusterEngine::cross(const geometry_msgs::msg::Point& O,
                            const geometry_msgs::msg::Point& A,
                            const geometry_msgs::msg::Point& B)
{
  return (A.x - O.x) * (B.y - O.y) - (A.y - O.y) * (B.x - O.x);
}

std::vector<geometry_msgs::msg::Point>
ClusterEngine::computeConvexHull(std::vector<geometry_msgs::msg::Point> pts)
{
  std::vector<geometry_msgs::msg::Point> H;
  if (pts.size() <= 1) return pts;

  std::sort(pts.begin(), pts.end(), [](const auto& a, const auto& b) {
    if (a.x < b.x) return true;
    if (a.x > b.x) return false;
    return a.y < b.y;
  });

  // Build lower hull
  for (const auto& p : pts) {
    while (H.size() >= 2 && cross(H[H.size() - 2], H.back(), p) <= 0.0) {
      H.pop_back();
    }
    H.push_back(p);
  }

  // Build upper hull
  size_t lower_size = H.size();
  for (int i = static_cast<int>(pts.size()) - 2; i >= 0; --i) {
    const auto& p = pts[static_cast<size_t>(i)];
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

std::vector<BlobCluster>
ClusterEngine::extract(nav2_costmap_2d::Costmap2D* costmap, unsigned char cost_threshold) const
{
  std::vector<BlobCluster> clusters;
  if (!costmap) {
    return clusters;
  }

  const unsigned int size_x = costmap->getSizeInCellsX();
  const unsigned int size_y = costmap->getSizeInCellsY();

  // visited[x][y]
  std::vector<std::vector<bool>> visited(size_x, std::vector<bool>(size_y, false));
  using Cell = std::pair<unsigned int, unsigned int>;

  auto floodFill = [&](unsigned int sx, unsigned int sy, std::vector<geometry_msgs::msg::Point>& out_points) {
    std::vector<Cell> stack;
    stack.emplace_back(sx, sy);

    while (!stack.empty()) {
      auto [cx, cy] = stack.back();
      stack.pop_back();

      if (visited[cx][cy]) continue;
      visited[cx][cy] = true;

      if (costmap->getCost(cx, cy) >= cost_threshold) {
        double wx, wy;
        costmap->mapToWorld(cx, cy, wx, wy);
        geometry_msgs::msg::Point p;
        p.x = wx;
        p.y = wy;
        p.z = 0.0;
        out_points.push_back(p);

        // 8-connected neighborhood
        for (int dx = -1; dx <= 1; ++dx) {
          for (int dy = -1; dy <= 1; ++dy) {
            if (dx == 0 && dy == 0) continue;
            int nx = static_cast<int>(cx) + dx;
            int ny = static_cast<int>(cy) + dy;
            if (nx >= 0 && ny >= 0 &&
                nx < static_cast<int>(size_x) &&
                ny < static_cast<int>(size_y) &&
                !visited[static_cast<size_t>(nx)][static_cast<size_t>(ny)]) {
              stack.emplace_back(static_cast<unsigned int>(nx),
                                 static_cast<unsigned int>(ny));
            }
          }
        }
      }
    }
  };

  for (unsigned int x = 0; x < size_x; ++x) {
    for (unsigned int y = 0; y < size_y; ++y) {
      if (!visited[x][y] && costmap->getCost(x, y) >= cost_threshold) {
        std::vector<geometry_msgs::msg::Point> pts;
        floodFill(x, y, pts);
        if (!pts.empty()) {
          BlobCluster cluster;
          cluster.points = std::move(pts);
          cluster.centroid = computeCentroid(cluster.points);
          cluster.boundary = computeConvexHull(cluster.points);
          clusters.push_back(std::move(cluster));
        }
      }
    }
  }

  return clusters;
}
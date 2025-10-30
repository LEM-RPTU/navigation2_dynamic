#include "tracker_engine.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

using Clock = std::chrono::steady_clock;

double TrackerEngine::dist(
  const geometry_msgs::msg::Point & a,
  const geometry_msgs::msg::Point & b)
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return std::sqrt(dx * dx + dy * dy);
}

std::vector<int> TrackerEngine::hungarian(
  const std::vector<std::vector<double>> & A)
{
  const size_t n = A.size();
  std::vector<double> u(n + 1, 0.0);
  std::vector<double> v(n + 1, 0.0);
  std::vector<int> p(n + 1, 0);
  std::vector<int> way(n + 1, 0);

  for (size_t i = 1; i <= n; ++i) {
    p[0] = static_cast<int>(i);
    std::vector<double> minv(n + 1, std::numeric_limits<double>::infinity());
    std::vector<char> used(n + 1, false);
    int j0 = 0;

    do {
      used[j0] = true;
      int i0 = p[j0];
      int j1 = 0;
      double delta = std::numeric_limits<double>::infinity();

      for (size_t j = 1; j <= n; ++j) {
        if (used[j]) {
          continue;
        }
        double cur = A[i0 - 1][j - 1] - u[i0] - v[j];
        if (cur < minv[j]) {
          minv[j] = cur;
          way[j] = j0;
        }
        if (minv[j] < delta) {
          delta = minv[j];
          j1 = static_cast<int>(j);
        }
      }

      for (size_t j = 0; j <= n; ++j) {
        if (used[j]) {
          u[p[j]] += delta;
          v[j] -= delta;
        } else {
          minv[j] -= delta;
        }
      }
      j0 = j1;
    } while (p[j0] != 0);

    do {
      int j1 = way[j0];
      p[j0] = p[j1];
      j0 = j1;
    } while (j0);
  }

  std::vector<int> assignment(n, -1);
  for (size_t j = 1; j <= n; ++j) {
    assignment[static_cast<size_t>(p[j] - 1)] = static_cast<int>(j - 1);
  }
  return assignment;
}

std::vector<ObstacleTrack> TrackerEngine::update(
  const std::vector<BlobCluster> & clusters)
{
  // Update delta time measurement
  double dt_sec = dt_fallback_;
  const auto now_tp = Clock::now();
  if (have_last_tp_) {
    std::chrono::duration<double> dt = now_tp - last_tp_;
    dt_sec = std::max(1e-6, std::min(dt.count(), 2.0));
  } else {
    have_last_tp_ = true;
  }
  last_tp_ = now_tp;
  last_dt_sec_ = dt_sec;

  retired_ids_.clear();

  const size_t M = tracks_.size();
  const size_t N = clusters.size();

  // First cycle: birth all detections as new tracks
  if (M == 0) {
    for (const auto & cluster : clusters) {
      ObstacleTrack t;
      t.id = next_id_++;
      t.current_position = cluster.centroid;
      t.predicted_next_position = cluster.centroid;
      t.polygon.points.clear();
      for (const auto & p : cluster.boundary) {
        geometry_msgs::msg::Point32 p32;
        p32.x = static_cast<float>(p.x);
        p32.y = static_cast<float>(p.y);
        p32.z = static_cast<float>(p.z);
        t.polygon.points.push_back(p32);
      }
      tracks_.push_back(std::move(t));
    }
    return tracks_;
  }

  // Build cost matrix: predicted_next_position → detection centroids
  const size_t K = std::max(M, N);
  const double BIG = 1e6;
  std::vector<std::vector<double>> C(K, std::vector<double>(K, BIG));

  for (size_t i = 0; i < M; ++i) {
    for (size_t j = 0; j < N; ++j) {
      C[i][j] = dist(tracks_[i].predicted_next_position, clusters[j].centroid);
    }
  }

  // Solve assignment problem (Hungarian algorithm)
  std::vector<int> row2col = hungarian(C);

  // Apply gating: only accept matches within gate_distance_
  std::vector<int> matched_det_for_track(M, -1);
  std::vector<bool> det_matched(N, false);

  for (size_t i = 0; i < M; ++i) {
    int j = row2col[i];
    if (j >= 0 && static_cast<size_t>(j) < N && C[i][j] <= gate_distance_) {
      matched_det_for_track[i] = j;
      det_matched[j] = true;
    }
  }

  // Birth new tracks for unmatched detections
  for (size_t j = 0; j < N; ++j) {
    if (!det_matched[j]) {
      ObstacleTrack t;
      t.id = next_id_++;
      t.current_position = clusters[j].centroid;
      t.predicted_next_position = clusters[j].centroid;
      t.polygon.points.clear();
      for (const auto & p : clusters[j].boundary) {
        geometry_msgs::msg::Point32 p32;
        p32.x = static_cast<float>(p.x);
        p32.y = static_cast<float>(p.y);
        p32.z = static_cast<float>(p.z);
        t.polygon.points.push_back(p32);
      }
      tracks_.push_back(std::move(t));
    }
  }

  // Update matched tracks, increment miss counter for unmatched
  for (size_t i = 0; i < M; ++i) {
    int j = matched_det_for_track[i];
    auto & t = tracks_[i];

    if (j < 0) {
      // Track not matched: increment missed counter
      t.missed++;
      continue;
    }

    // Matched: update with new detection
    t.current_position = clusters[j].centroid;
    t.missed = 0;

    t.polygon.points.clear();
    for (const auto & p : clusters[j].boundary) {
      geometry_msgs::msg::Point32 p32;
      p32.x = static_cast<float>(p.x);
      p32.y = static_cast<float>(p.y);
      p32.z = static_cast<float>(p.z);
      t.polygon.points.push_back(p32);
    }
  }

  // Retire tracks that have been unmatched for too long
  tracks_.erase(
    std::remove_if(
      tracks_.begin(), tracks_.end(),
      [&](const ObstacleTrack & t) {
        if (t.missed > max_missed_) {
          retired_ids_.push_back(t.id);
          return true;
        }
        return false;
      }),
    tracks_.end());

  return tracks_;
}

void TrackerEngine::updatePredictedPositions(
  const std::unordered_map<int64_t, geometry_msgs::msg::Point> & predicted_t1)
{
  for (auto & t : tracks_) {
    auto it = predicted_t1.find(t.id);
    if (it != predicted_t1.end()) {
      t.predicted_next_position = it->second;
    }
    // Else: keep current position as fallback
  }
}
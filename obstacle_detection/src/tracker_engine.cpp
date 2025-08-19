#include "tracker_engine.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

using Clock = std::chrono::steady_clock;

double TrackerEngine::dist(const geometry_msgs::msg::Point& a,
                           const geometry_msgs::msg::Point& b)
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return std::sqrt(dx * dx + dy * dy);
}

std::vector<int> TrackerEngine::hungarian(const std::vector<std::vector<double>>& A)
{
  const size_t n = A.size();
  std::vector<double> u(n + 1, 0), v(n + 1, 0);
  std::vector<int> p(n + 1, 0), way(n + 1, 0);

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
        if (used[j]) continue;
        double cur = A[i0 - 1][j - 1] - u[i0] - v[j];
        if (cur < minv[j]) { minv[j] = cur; way[j] = j0; }
        if (minv[j] < delta) { delta = minv[j]; j1 = static_cast<int>(j); }
      }
      for (size_t j = 0; j <= n; ++j) {
        if (used[j]) { u[p[j]] += delta; v[j] -= delta; }
        else { minv[j] -= delta; }
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
  return assignment; // row i -> col assignment[i]
}

void TrackerEngine::cap_history(std::vector<geometry_msgs::msg::Point>& hist)
{
  if (history_len_ == 0) return;
  if (hist.size() > history_len_) {
    const std::size_t drop = hist.size() - history_len_;
    hist.erase(hist.begin(), hist.begin() + static_cast<std::ptrdiff_t>(drop));
  }
}

TrackingResult TrackerEngine::update(const std::vector<BlobCluster>& clusters)
{
  // Compute dt
  double dt_sec = dt_fallback_;
  const auto now = Clock::now();
  if (have_last_tp_) {
    std::chrono::duration<double> dt = now - last_tp_;
    dt_sec = dt.count();
    if (!std::isfinite(dt_sec) || dt_sec <= 1e-6) dt_sec = dt_fallback_;
    // Optional clamp to avoid huge spikes
    if (dt_sec > 2.0) dt_sec = 2.0;
  } else {
    have_last_tp_ = true;
  }
  last_tp_ = now;
  last_dt_sec_ = dt_sec;

  TrackingResult out;

  const size_t M = tracks_.size();
  const size_t N = clusters.size();

  // If no existing tracks, birth all clusters
  if (M == 0) {
    out.assignments.reserve(N);
    for (size_t j = 0; j < N; ++j) {
      Track t;
      t.id = next_id_++;
      t.history.push_back(clusters[j].centroid);
      cap_history(t.history);
      t.pred_t1 = clusters[j].centroid; // one-cycle-lag stub
      t.velocity = geometry_msgs::msg::Vector3(); // 0 m/s
      tracks_.push_back(t);

      ClusterAssignment a;
      a.cluster_index = j;
      a.id = t.id;
      a.cost = 0.0;
      a.new_track = true;
      a.velocity = t.velocity;
      out.assignments.push_back(a);
    }
    return out;
  }

  // Build predictions for rows (tracks) and cost matrix against detections (clusters)
  const size_t K = std::max(M, N);
  const double BIG = 1e6;
  std::vector<std::vector<double>> C(K, std::vector<double>(K, BIG));

  for (size_t i = 0; i < M; ++i) {
    const geometry_msgs::msg::Point& pr = tracks_[i].pred_t1;
    for (size_t j = 0; j < N; ++j) {
      C[i][j] = dist(pr, clusters[j].centroid);
    }
  }

  // Hungarian on square KxK
  std::vector<int> assign_rows = hungarian(C);

  // Track which detections are matched
  std::vector<bool> det_matched(N, false);

  // Accept matches within gate
  for (size_t i = 0; i < M; ++i) {
    int j = assign_rows[i];
    if (j < 0 || static_cast<size_t>(j) >= N) continue;
    const double d = C[i][static_cast<size_t>(j)];
    if (d <= gate_distance_) {
      // Update track i with detection j
      auto& t = tracks_[i];
      t.missed = 0;

      if (!t.history.empty()) {
        const auto& prev = t.history.back();
        geometry_msgs::msg::Vector3 v;
        const double inv_dt = (dt_sec > 1e-6) ? (1.0 / dt_sec) : 0.0;
        v.x = (clusters[static_cast<size_t>(j)].centroid.x - prev.x) * inv_dt;
        v.y = (clusters[static_cast<size_t>(j)].centroid.y - prev.y) * inv_dt;
        v.z = 0.0;
        t.velocity = v; // meters/second
      }

      t.history.push_back(clusters[static_cast<size_t>(j)].centroid);
      cap_history(t.history);

      // One-cycle-lag prediction stub
      t.pred_t1 = t.history.back();

      det_matched[static_cast<size_t>(j)] = true;

      ClusterAssignment a;
      a.cluster_index = static_cast<size_t>(j);
      a.id = t.id;
      a.cost = d;
      a.new_track = false;
      a.velocity = t.velocity;
      out.assignments.push_back(a);
    }
  }

  // Age unmatched tracks
  for (size_t i = 0; i < M; ++i) {
    bool matched = false;
    for (const auto& a : out.assignments) {
      if (tracks_[i].id == a.id) { matched = true; break; }
    }
    if (!matched) {
      tracks_[i].missed++;
    }
  }

  // Birth new tracks for unmatched detections
  for (size_t j = 0; j < N; ++j) {
    if (!det_matched[j]) {
      Track t;
      t.id = next_id_++;
      t.history.push_back(clusters[j].centroid);
      cap_history(t.history);
      t.pred_t1 = clusters[j].centroid;
      t.velocity = geometry_msgs::msg::Vector3(); // 0 m/s
      tracks_.push_back(t);

      ClusterAssignment a;
      a.cluster_index = j;
      a.id = t.id;
      a.cost = BIG;
      a.new_track = true;
      a.velocity = t.velocity;
      out.assignments.push_back(a);
    }
  }

  // Retire aged-out tracks
  tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(),
                               [&](const Track& t){
                                 if (t.missed > max_missed_) {
                                   out.retired_tracks.push_back(t.id);
                                   return true;
                                 }
                                 return false;
                               }),
                tracks_.end());

  // Order assignments by cluster index for easier consumption
  std::sort(out.assignments.begin(), out.assignments.end(),
            [](const ClusterAssignment& A, const ClusterAssignment& B){
              return A.cluster_index < B.cluster_index;
            });

  return out;
}
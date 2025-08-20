#include "tracker_engine.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_map>

using Clock = std::chrono::steady_clock;

double TrackerEngine::dist(const geometry_msgs::msg::Point &a,
                           const geometry_msgs::msg::Point &b)
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return std::sqrt(dx * dx + dy * dy);
}

std::vector<int> TrackerEngine::hungarian(const std::vector<std::vector<double>> &A)
{
  const size_t n = A.size();
  std::vector<double> u(n + 1, 0), v(n + 1, 0);
  std::vector<int> p(n + 1, 0), way(n + 1, 0);

  for (size_t i = 1; i <= n; ++i)
  {
    p[0] = static_cast<int>(i);
    std::vector<double> minv(n + 1, std::numeric_limits<double>::infinity());
    std::vector<char> used(n + 1, false);
    int j0 = 0;
    do
    {
      used[j0] = true;
      int i0 = p[j0];
      int j1 = 0;
      double delta = std::numeric_limits<double>::infinity();
      for (size_t j = 1; j <= n; ++j)
      {
        if (used[j])
          continue;
        double cur = A[i0 - 1][j - 1] - u[i0] - v[j];
        if (cur < minv[j])
        {
          minv[j] = cur;
          way[j] = j0;
        }
        if (minv[j] < delta)
        {
          delta = minv[j];
          j1 = static_cast<int>(j);
        }
      }
      for (size_t j = 0; j <= n; ++j)
      {
        if (used[j])
        {
          u[p[j]] += delta;
          v[j] -= delta;
        }
        else
        {
          minv[j] -= delta;
        }
      }
      j0 = j1;
    } while (p[j0] != 0);
    do
    {
      int j1 = way[j0];
      p[j0] = p[j1];
      j0 = j1;
    } while (j0);
  }

  std::vector<int> assignment(n, -1);
  for (size_t j = 1; j <= n; ++j)
  {
    assignment[static_cast<size_t>(p[j] - 1)] = static_cast<int>(j - 1);
  }
  return assignment; // row i -> col assignment[i]
}

void TrackerEngine::cap_history(std::vector<geometry_msgs::msg::Point> &hist)
{
  if (history_len_ == 0)
    return;
  if (hist.size() > history_len_)
  {
    const std::size_t drop = hist.size() - history_len_;
    hist.erase(hist.begin(), hist.begin() + static_cast<std::ptrdiff_t>(drop));
  }
}

TrackerPredictionFrame TrackerEngine::update(const std::vector<BlobCluster> &clusters)
{
  // 0) dt bookkeeping
  double dt_sec = dt_fallback_;
  const auto now_tp = Clock::now();
  if (have_last_tp_)
  {
    std::chrono::duration<double> dt = now_tp - last_tp_;
    dt_sec = dt.count();
    if (!std::isfinite(dt_sec) || dt_sec <= 1e-6)
      dt_sec = dt_fallback_;
    if (dt_sec > 2.0)
      dt_sec = 2.0;
  }
  else
  {
    have_last_tp_ = true;
  }
  last_tp_ = now_tp;
  last_dt_sec_ = dt_sec;

  TrackerPredictionFrame frame;
  frame.stamp = rclcpp::Clock().now();

  const size_t M = tracks_.size();
  const size_t N = clusters.size();
  const size_t K = std::max(M, N);
  const double BIG = 1e6;

  std::vector<ClusterAssignment> assignments;
  assignments.reserve(std::max(M, N));

  // 1) If no tracks yet, birth all
  if (M == 0)
  {
    frame.tracks.reserve(N);
    for (size_t j = 0; j < N; ++j)
    {
      Track t;
      t.id = next_id_++;
      t.history.push_back(clusters[j].centroid);
      cap_history(t.history);
      t.pred_t1 = t.history.back();
      t.velocity = geometry_msgs::msg::Vector3(); // 0 m/s
      tracks_.push_back(t);

      ClusterAssignment a{j, t.id, 0.0, true};
      assignments.push_back(a);

      ObstacleTrack ot;
      ot.id = t.id;
      ot.position = {clusters[j].centroid}; // current at [0]
      ot.history = t.history;               // includes current at back
      ot.velocity = t.velocity;

      geometry_msgs::msg::Vector3 heading{};
      ot.heading = heading;
      ot.is_dynamic = false;

      // hull
      ot.polygon.points.reserve(clusters[j].boundary.size());
      for (const auto &p : clusters[j].boundary)
      {
        geometry_msgs::msg::Point32 p32;
        p32.x = static_cast<float>(p.x);
        p32.y = static_cast<float>(p.y);
        p32.z = static_cast<float>(p.z);
        ot.polygon.points.push_back(p32);
      }

      frame.tracks.push_back(std::move(ot));
    }
    frame.assignments = std::move(assignments);
    return frame;
  }

  // 2) Build cost from pred_t1 to detections and run Hungarian
  std::vector<std::vector<double>> C(K, std::vector<double>(K, BIG));
  for (size_t i = 0; i < M; ++i)
  {
    const auto &pr = tracks_[i].pred_t1;
    for (size_t j = 0; j < N; ++j)
      C[i][j] = dist(pr, clusters[j].centroid);
  }
  std::vector<int> row2col = hungarian(C);

  // 3) Collect matches within gate and flags
  std::vector<int> matched_det_for_track(M, -1);
  std::vector<bool> det_matched(N, false);
  for (size_t i = 0; i < M; ++i)
  {
    int j = row2col[i];
    if (j >= 0 && static_cast<size_t>(j) < N)
    {
      const double d = C[i][static_cast<size_t>(j)];
      if (d <= gate_distance_)
      {
        matched_det_for_track[i] = j;
        det_matched[static_cast<size_t>(j)] = true;
      }
    }
  }

  // 4) Birth unmatched detections
  for (size_t j = 0; j < N; ++j)
  {
    if (!det_matched[j])
    {
      Track t;
      t.id = next_id_++;
      t.history.push_back(clusters[j].centroid);
      cap_history(t.history);
      t.pred_t1 = t.history.back();
      t.velocity = geometry_msgs::msg::Vector3(); // 0 m/s
      tracks_.push_back(t);

      ClusterAssignment a{j, t.id, BIG, true};
      assignments.push_back(a);
    }
  }

  // 5) Age unmatched tracks (no observation this tick)
  for (size_t i = 0; i < M; ++i)
  {
    if (matched_det_for_track[i] < 0)
    {
      tracks_[i].missed++;
    }
  }

  // 6) Kinematics update for matched tracks AFTER appending new observation
  const double inv_dt = (dt_sec > 1e-6) ? (1.0 / dt_sec) : 0.0;
  for (size_t i = 0; i < M; ++i)
  {
    int j = matched_det_for_track[i];
    if (j < 0)
      continue;

    auto &t = tracks_[i];
    const auto &z = clusters[static_cast<size_t>(j)].centroid;

    // append new observation
    geometry_msgs::msg::Vector3 v{};
    if (!t.history.empty())
    {
      const auto &prev = t.history.back();
      v.x = (z.x - prev.x) * inv_dt;
      v.y = (z.y - prev.y) * inv_dt;
      v.z = 0.0;
    }
    t.history.push_back(z);
    cap_history(t.history);
    t.velocity = v;
    t.missed = 0;

    // one-cycle-lag stub
    t.pred_t1 = t.history.back();

    // emit assignment now (WITHOUT velocity, it's in track data)
    ClusterAssignment a;
    a.cluster_index = static_cast<size_t>(j);
    a.id = t.id;
    a.cost = C[i][static_cast<size_t>(j)];
    a.new_track = false;
    assignments.push_back(a);
  }

  // 7) Retire aged-out tracks
  std::vector<int64_t> retired;
  tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(),
                               [&](const Track &t)
                               {
                                 if (t.missed > max_missed_)
                                 {
                                   retired.push_back(t.id);
                                   return true;
                                 }
                                 return false;
                               }),
                tracks_.end());

  // 8) Package frame tracks (current snapshot)
  frame.tracks.reserve(assignments.size());
  for (const auto &a : assignments)
  {
    // find current centroid and hull from cluster
    const auto &c = clusters[a.cluster_index];

    // find matching internal track by id
    const Track *ti = nullptr;
    for (const auto &t : tracks_)
    {
      if (t.id == a.id)
      {
        ti = &t;
        break;
      }
    }
    if (!ti)
      continue;

    ObstacleTrack ot;
    ot.id = ti->id;
    ot.position = {c.centroid};
    ot.history = ti->history;
    ot.velocity = ti->velocity;

    geometry_msgs::msg::Vector3 heading{};
    const double n = std::hypot(ot.velocity.x, ot.velocity.y);
    if (n > 1e-6)
    {
      heading.x = ot.velocity.x / n;
      heading.y = ot.velocity.y / n;
    }
    else
    {
      heading.x = heading.y = 0.0;
    }
    heading.z = 0.0;
    ot.heading = heading;

    // Mark as dynamic if significant velocity
    ot.is_dynamic = (n > 0.2); // > 0.2 m/s

    // Add polygon from cluster
    ot.polygon.points.reserve(clusters[a.cluster_index].boundary.size());
    for (const auto &p : clusters[a.cluster_index].boundary)
    {
      geometry_msgs::msg::Point32 p32;
      p32.x = static_cast<float>(p.x);
      p32.y = static_cast<float>(p.y);
      p32.z = static_cast<float>(p.z);
      ot.polygon.points.push_back(p32);
    }

    frame.tracks.push_back(std::move(ot));
  }

  // Store assignments in the frame for downstream consumption
  frame.assignments = std::move(assignments);
  frame.retired_tracks = std::move(retired);

  return frame;
}
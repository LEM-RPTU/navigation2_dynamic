#pragma once

#include <vector>
#include <cstdint>
#include <unordered_map>
#include <chrono>
#include <geometry_msgs/msg/point.hpp>
#include "types.hpp"

class TrackerEngine
{
public:
  explicit TrackerEngine(double gate_distance = 3.0,
                         int max_missed = 3,
                         double dt_fallback = 0.2)
  : gate_distance_(gate_distance),
    max_missed_(max_missed),
    dt_fallback_(dt_fallback) {}

  void set_gate_distance(double d) { gate_distance_ = d; }
  void set_max_missed(int m)       { max_missed_ = m; }
  void set_dt_fallback(double d)   { dt_fallback_ = d; }

  double last_dt() const { return last_dt_sec_; }

  // Update tracks with new detections
  // Returns: active tracks (with IDs assigned to blobs)
  std::vector<ObstacleTrack> update(const std::vector<BlobCluster>& clusters);

  // Update predicted positions from predictor (for next cycle's Hungarian)
  void updatePredictedPositions(const std::unordered_map<int64_t, geometry_msgs::msg::Point>& predicted_t1);

  // Get retired track IDs from last update
  const std::vector<int64_t>& get_retired_ids() const { return retired_ids_; }

private:
  static std::vector<int> hungarian(const std::vector<std::vector<double>>& cost);
  static double dist(const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b);

  double gate_distance_;
  int max_missed_;
  double dt_fallback_;
  int64_t next_id_ = 1;

  // Persistent state
  std::vector<ObstacleTrack> tracks_;
  std::vector<int64_t> retired_ids_;  // Populated in update()

  // Time bookkeeping
  std::chrono::steady_clock::time_point last_tp_;
  bool have_last_tp_ = false;
  double last_dt_sec_ = 0.0;
};
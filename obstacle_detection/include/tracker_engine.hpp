#pragma once

#include <vector>
#include <cstdint>
#include <limits>
#include <chrono>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include "types.hpp"

class TrackerEngine
{
public:
  explicit TrackerEngine(double gate_distance = 3.0,
                         int max_missed = 3,
                         std::size_t history_len = 5,
                         double dt_fallback = 0.2)
  : gate_distance_(gate_distance),
    max_missed_(max_missed),
    history_len_(history_len),
    dt_fallback_(dt_fallback) {}

  void set_gate_distance(double d) { gate_distance_ = d; }
  void set_max_missed(int m)       { max_missed_ = m; }
  void set_history_len(std::size_t n) { history_len_ = n; }
  void set_dt_fallback(double d)   { dt_fallback_ = d; }

  double last_dt() const { return last_dt_sec_; }

  // Update persistent tracks with current clusters.
  // Returns both assignment info and full track data.
  TrackerPredictionFrame update(const std::vector<BlobCluster>& clusters);

private:
  // Hungarian on a square matrix. Returns assignment row->col.
  static std::vector<int> hungarian(const std::vector<std::vector<double>>& cost);

  static double dist(const geometry_msgs::msg::Point& a,
                     const geometry_msgs::msg::Point& b);

  struct Track
  {
    int64_t id;
    int missed{0};
    std::vector<geometry_msgs::msg::Point> history;  // most-recent at back
    geometry_msgs::msg::Point pred_t1;               // one-cycle-lag stub (== last centroid)
    geometry_msgs::msg::Vector3 velocity;            // meters/second
  };

  // Parameters
  double gate_distance_;
  int max_missed_;
  std::size_t history_len_;
  double dt_fallback_;
  int64_t next_id_ = 1;

  // State
  std::vector<Track> tracks_;

  // Time bookkeeping (for dt)
  std::chrono::steady_clock::time_point last_tp_;
  bool have_last_tp_ = false;
  double last_dt_sec_ = 0.0;

  // Helpers
  void cap_history(std::vector<geometry_msgs::msg::Point>& hist);
};
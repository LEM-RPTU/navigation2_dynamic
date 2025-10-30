#pragma once

#include <vector>
#include <cstdint>
#include <unordered_map>
#include <chrono>

#include <geometry_msgs/msg/point.hpp>

#include "types.hpp"

/**
 * @class TrackerEngine
 * @brief Multi-hypothesis tracker using Hungarian algorithm for data association
 * 
 * Maintains persistent tracks across detection cycles, performs gated Hungarian
 * matching between predicted positions and new detections, handles track birth
 * and retirement based on missed detection counters.
 */
class TrackerEngine
{
public:
  /**
   * @brief Construct tracker with default parameters
   * @param gate_distance Maximum distance for valid track-detection match (meters)
   * @param max_missed Maximum consecutive missed detections before track retirement
   * @param dt_fallback Fallback timestep if timing measurement unavailable (seconds)
   */
  explicit TrackerEngine(
    double gate_distance = 3.0,
    int max_missed = 3,
    double dt_fallback = 0.2)
  : gate_distance_(gate_distance),
    max_missed_(max_missed),
    dt_fallback_(dt_fallback) {}

  void set_gate_distance(double d) {gate_distance_ = d;}
  void set_max_missed(int m) {max_missed_ = m;}
  void set_dt_fallback(double d) {dt_fallback_ = d;}

  double last_dt() const {return last_dt_sec_;}

  /**
   * @brief Update tracks with new detections using Hungarian matching
   * @param clusters New blob detections from current cycle
   * @return Active tracks with assigned IDs and updated states
   * 
   * Performs:
   * 1. Hungarian assignment (predicted_pos ↔ detection_centroid)
   * 2. Gating (reject matches > gate_distance)
   * 3. Birth new tracks for unmatched detections
   * 4. Update matched tracks
   * 5. Retire tracks with missed > max_missed
   */
  std::vector<ObstacleTrack> update(const std::vector<BlobCluster> & clusters);

  /**
   * @brief Update predicted positions from external predictor
   * @param predicted_t1 Map of track_id → predicted position at t+1
   * 
   * Sets predicted_next_position for each track (used in next cycle's Hungarian).
   * Implements closed-loop prediction feedback.
   */
  void updatePredictedPositions(
    const std::unordered_map<int64_t, geometry_msgs::msg::Point> & predicted_t1);

  /**
   * @brief Get track IDs retired in last update() call
   * @return Vector of retired track IDs
   */
  const std::vector<int64_t> & get_retired_ids() const {return retired_ids_;}

private:
  /**
   * @brief Hungarian algorithm for optimal assignment
   * @param cost Square cost matrix (n×n)
   * @return Assignment vector: assignment[i] = j means row i → column j
   */
  static std::vector<int> hungarian(const std::vector<std::vector<double>> & cost);

  /**
   * @brief Euclidean distance between two points
   * @param a First point
   * @param b Second point
   * @return Distance in meters
   */
  static double dist(
    const geometry_msgs::msg::Point & a,
    const geometry_msgs::msg::Point & b);

  // Configuration parameters
  double gate_distance_;  // Gating threshold (meters)
  int max_missed_;        // Miss tolerance before retirement
  double dt_fallback_;    // Default timestep (seconds)
  int64_t next_id_ = 1;   // Next track ID to assign

  // Persistent state
  std::vector<ObstacleTrack> tracks_;      // Active tracks
  std::vector<int64_t> retired_ids_;       // Tracks retired in last update

  // Time bookkeeping for dt measurement
  std::chrono::steady_clock::time_point last_tp_;
  bool have_last_tp_ = false;
  double last_dt_sec_ = 0.0;
};
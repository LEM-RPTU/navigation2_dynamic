# navigation2\_dynamic

Dynamic obstacle **detection → tracking → prediction** for Nav2 (ROS 2).
This repo provides:

* **Foreground mask layer** for Costmap2D (separates dynamic foreground from inflated static background)
* **Detection node**: Obstacle detection and persistent tracking with Hungarian assignment
* **Predictor node**: Constant velocity prediction for short-horizon trajectory forecasts
* **Custom messages** for obstacle exchange (topic-based architecture)
* **Visualization node** for RViz markers (separate from core processing)

---

## Repository layout

```
navigation2_dynamic/
├── foreground_mask_layer/         # costmap2d plugin (C++)
├── nav2_dynamic_interface/        # msgs (ObstacleArray, Obstacle)
├── obstacle_detection/            # detection + tracking node (C++)
│   ├── src/detection_node.cpp     # main detection/tracking node
│   ├── src/obstacle_viz_node.cpp  # visualization node
│   ├── src/cluster_engine.cpp     # blob extraction
│   └── src/tracker_engine.cpp     # Hungarian tracking
└── obstacle_predictor/            # predictor node (Python)
    └── obstacle_predictor/
        └── predictor_node_cv.py      # CV prediction with history
```

---

## High-level flow

1. **Foreground mask** isolates dynamic cells from obstacle layer (fast) vs inflated static map (slow).
2. **ClusterEngine** extracts blobs (centroid + convex hull).
3. **TrackerEngine** keeps persistent IDs (Hungarian assignment, gating, history).
4. **Detection node** publishes current detections on `obstacles_detected` topic.
5. **Predictor node** subscribes to detections, maintains history per ID, computes predictions, and publishes full trajectories on `obstacles_array`.
6. **Detection node** subscribes to `obstacles_array` to update tracker with predicted positions for next-cycle matching.
7. **Visualization node** generates RViz markers for hulls, centroids, IDs, trajectories, covariance ellipses, and velocity arrows.

---

## Packages

### 1) `foreground_mask_layer` (C++)

Costmap2D plugin (`nav2_costmap_2d::Layer`) that:

* Subscribes to a static map, **inflates** it by a configurable radius,
* Compares rolling obstacle costs vs inflated static background,
* Flags **foreground** (dynamic) cells (optionally writing them back to the master grid).

**Key params (YAML)**

```yaml
foreground_mask_layer:
  plugin: "foreground_mask::ForegroundMaskLayer"
  map_topic: "/map"            # static map
  inflation_radius: 20         # buffer around static obstacles
  observation_sources: scan    # pass-through to ObstacleLayer
```

### 2) `nav2_dynamic_interface`

Interfaces shared across nodes.

**Messages**

* `Obstacle.msg`

  * `std_msgs/Header header`
  * `unique_identifier_msgs/UUID uuid`
  * `int64 id`                                    # Persistent track ID
  * `builtin_interfaces/Duration delta_time`      # Fixed timestep between predictions
  * `geometry_msgs/PoseWithCovariance[] position` # position[0]=current, [1..N]=predictions
  * `geometry_msgs/TwistWithCovariance[] velocity`# velocity[0..N] aligned with positions
  * `geometry_msgs/Polygon polygon`               # Convex hull boundary

* `ObstacleArray.msg`

  * `std_msgs/Header header`
  * `Obstacle[] obstacles`

### 3) `obstacle_detection` (C++)

Executable: `detection_node`

* Brings up a dedicated `Costmap2DROS`
* Clusters blobs (8-connectivity), centroids, convex hull (monotone chain)
* Tracks across ticks (Hungarian assignment, gating radius, missed counts)
* Publishes current detections and subscribes to predictions for tracker updates

**Topics**

* **Pub:** `obstacles_detected` (`nav2_dynamic_interface/ObstacleArray`) - Current detections only (position[0])
* **Sub:** `obstacles_array` (`nav2_dynamic_interface/ObstacleArray`) - Full predictions from predictor
* **Sub (via Costmap2DROS/ObstacleLayer):** scans, TF, map, etc.

Detection node now:
1. Publishes detections with only `position[0]` (current state)
2. Subscribes to predictor's output to update tracker's `predicted_next_position` for improved Hungarian matching
3. Does NOT publish final obstacle array (predictor owns that responsibility)

**Selected params**

```yaml
detection_node:
  ros__parameters:
    tracking:
      gate_distance: 1.5        # Max distance (m) for valid track-detection match
      max_missed: 3             # Max consecutive missed detections before track retirement
    
    prediction:
      delta_time: 0.2           # Time step between predictions (seconds, used for msg field)
```

### 4) `obstacle_predictor` (Python / rclpy)

Executable: `predictor_node_cv`

Topic-based pub/sub architecture with **constant velocity model only**.

* Subscribes to `obstacles_detected` (current detections)
* Maintains per-ID history buffer (configurable window)
* Computes velocity from full history (first → last point)
* Publishes `obstacles_array` with current + predicted positions

**Topics**

* **Sub:** `obstacles_detected` - Current detections from detection_node
* **Pub:** `obstacles_array` - Full obstacle trajectories (current + predictions)

**Selected params**

```yaml
predictor_node:
  ros__parameters:
    # History management
    history_window: 20          # Max number of past poses to store per track
    cleanup_threshold: 10       # Remove tracks not seen for N cycles
    
    # Velocity estimation (uses full history: first → last point)
    min_history_length: 3       # Min poses required before estimating non-zero velocity
    
    # Prediction output
    prediction_steps: 5         # Number of future steps to predict
    prediction_dt: 0.2          # Time between prediction steps (seconds)
```

### 5) `obstacle_visualization` (C++)

Executable: `obstacle_viz_node`

* Subscribes to `obstacles_array` and generates RViz-friendly visualization
* Publishes marker arrays for centroids, trajectories, hulls, velocity arrows, covariance ellipses, track IDs
* Decouples visualization from core detection/tracking for better resource management

**Topics**
* **Sub:** `obstacles_array`
* **Pub:** `cluster_markers` (visualization_msgs/MarkerArray)

**Marker types:**
- Centroid spheres (color-coded by track ID)
- Track ID labels
- Predicted trajectory line strips
- Convex hull boundaries
- Covariance ellipses (growing with prediction horizon)
- Velocity arrows (color-coded by speed: green→yellow→red)

---

## Build & install

```bash
# create a workspace if needed
mkdir -p ~/ws_nav2dyn/src && cd ~/ws_nav2dyn/src

# clone your repo
git clone -b dev https://github.com/LEM-RPTU/navigation2_dynamic.git navigation2_dynamic

# build
cd ..
colcon build --symlink-install \
  --packages-select \
    foreground_mask_layer nav2_dynamic_interface obstacle_detection obstacle_predictor

# source
source install/setup.bash
```

**Dependencies**

* ROS 2 Humble+
* `nav2_costmap_2d`, `rclcpp(_lifecycle)`, `geometry_msgs`, `visualization_msgs`, `unique_identifier_msgs`
* Python node: `rclpy` (no external dependencies like numpy)

---

## Launch

A combined launch starts the detection node, predictor node, and visualization under a namespace:

```bash
ros2 launch obstacle_detection dynamic_obstacle.launch.py \
  namespace:=fleet/robot1 \
  params_file:=/path/to/obstacle_predictor.yaml
```

**Or launch nodes individually:**

```bash
# Terminal 1: Detection node
ros2 run obstacle_detection detection_node --ros-args \
  -r __ns:=/robot1 \
  --params-file /path/to/obstacle_predictor.yaml

# Terminal 2: Predictor node
ros2 run obstacle_predictor predictor_node --ros-args \
  -r __ns:=/robot1 \
  --params-file /path/to/obstacle_predictor.yaml

# Terminal 3: Visualization node
ros2 run obstacle_detection obstacle_viz_node --ros-args \
  -r __ns:=/robot1
```

Expected outputs:

* `/<ns>/obstacles_detected` (detections only)
* `/<ns>/obstacles_array` (full predictions)
* `/<ns>/cluster_markers` (visualization)

**Frame IDs**

* Costmap uses `map` as `global_frame`; ensure TF `map↔odom` exists.
* Messages carry the costmap's `global_frame_id`.

---

## Configuration tips

* **Foreground vs background:** Pick a sufficiently **large `inflation_radius`** for the static map to avoid mislabeling static obstacles as dynamic due to pose drift with **larger COSTMAP**.
* **Tracking gate:** `tracking.gate_distance` bounds associations; tune for your costmap resolution and dynamics.
* **History length:** `history_window` caps memory usage; `min_history_length` ensures stable velocity estimates.
* **Prediction timestep:** Both nodes should use the same `prediction_dt` value (0.2s default).
* **Detection rate:** Detection node runs at 200ms (5Hz); predictor processes asynchronously.
* **Visualization:** Run on a separate executor to avoid slowing detection/tracking.

---

## Visualization (RViz)

Add these displays:

* **MarkerArray** on `/<ns>/obstacle_markers`:
  - Namespace filters: `centroids`, `labels`, `hulls`, `trajectories`, `covariance`, `velocity`
* **Costmap** layers to confirm foreground masking
* Set fixed frame to `map`

**Marker color coding:**
- Centroids: Hashed by track ID (persistent colors)
- Velocity arrows: Green (slow) → Yellow → Red (fast)
- Trajectories: Predicted path line strips

---

## Message contract (updated)

```text
Topic: obstacles_detected (nav2_dynamic_interface/ObstacleArray)
  Published by: detection_node
  Consumed by: predictor_node
  
  header.stamp, header.frame_id
  obstacles[*]:
    id, uuid
    position[0]        # Current centroid only
    velocity           # Empty (not computed by detector)
    polygon            # Convex hull
    delta_time         # Set to 0 (ignored)

Topic: obstacles_array (nav2_dynamic_interface/ObstacleArray)
  Published by: predictor_node
  Consumed by: detection_node (for tracker updates), obstacle_viz_node
  
  header.stamp, header.frame_id
  obstacles[*]:
    id, uuid
    delta_time         # Fixed prediction timestep (e.g., 0.2s)
    position[0]        # Current position (copied from detection)
    position[1..N]     # N predicted future positions
    velocity[0..N]     # Estimated/predicted velocities (aligned with positions)
    polygon            # Convex hull
    covariance fields  # Pose/velocity uncertainty (grows with horizon)
```

---

## Development notes

* **C++ standards**: Plugin uses C++14; detection node uses C++17.
* **Assignment**: Hungarian (square cost matrix, distance on centroids), with gating.
* **Velocity estimation**: Predictor uses full history (first → last point) for smoothing.
* **Threading**:
  * `Costmap2DROS` spins in its own thread.
  * Detection node timer runs at 200ms (5Hz).
  * Predictor uses `MultiThreadedExecutor` with `ReentrantCallbackGroup` for concurrent topic processing.
  * No worker queues or async service calls (topic-based is inherently async).

* **Topic flow**:
  ```
  detection_node → obstacles_detected → predictor_node
                            ↑               ↓
  obstacle_viz_node ← obstacles_array ──────┘
  ```

---

## Known limitations

* **Only constant velocity model** — No Kalman Filter or VAR; covariances are heuristic (scatter-based).
* **Integer IDs** — UUID field exists but not fully utilized; track IDs are simple monotonic integers.
* **No ego motion compensation** — Moving robot doesn't filter its own motion from obstacle velocity estimates.

---

## Roadmap

* **Full UUID support** - Replace integer IDs with proper UUID implementation for robust tracking across nodes
* **Ego motion compensation** - Implement proper ego-vehicle motion filtering to improve dynamic object identification
* **Kalman Filter predictor** - Implement proper CV/CTRV Kalman filter with tuned process/measurement noise
* **VAR predictor** - Implement Vector Auto-Regressive with proper configurations
* **Unit/integration tests** - Add CI with colcon test

---

## License

* **Top-level**: Apache-2.0 (see `LICENSE`).
* Packages may declare **BSD-3-Clause** or Apache-2.0 in their `package.xml`.
  Always check the package's own `package.xml` for the definitive license.

---

## Acknowledgments

Based on Nav2 and ROS 2 ecosystems. Thanks to the contributors and advisors involved in design discussions and review.

---

## Contact

- Maintainer: **[Riyan Cyriac Jose](https://github.com/joseriyancyriac)**
- Contribution: **[Eric Schoeneberg](https://github.com/Scoeerg)**

- Issues: [GitHub Issues](https://github.com/LEM-RPTU/navigation2_dynamic/issues)

---

## Appendix: Minimal YAML snippet

```yaml
# Dynamic obstacle costmap configuration
/**/dynamic_obstacle_costmap:
  dynamic_obstacle_costmap:
    ros__parameters:
      update_frequency: 5.0
      global_frame: map
      robot_base_frame: base_footprint
      rolling_window: true
      width: 20
      height: 20
      resolution: 0.1
      plugins: ["foreground_mask_layer", "denoise_layer", "inflation_layer"]
      
      foreground_mask_layer:
        plugin: "foreground_mask::ForegroundMaskLayer"
        map_topic: "/map"
        inflation_radius: 15
        observation_sources: scan
        scan:
          topic: /scan
          data_type: "LaserScan"
      
      denoise_layer:
        plugin: "nav2_costmap_2d::DenoiseLayer"
        minimal_group_size: 2
      
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        inflation_radius: 0.3

# Detection node configuration
/**/detection_node:
  ros__parameters:
    tracking:
      gate_distance: 1.5        # Max distance (m) for valid track-detection match
      max_missed: 3             # Max consecutive missed detections before track retirement
    
    prediction:
      delta_time: 0.2           # Time step between predictions (seconds)

# Predictor node configuration
/**/predictor_node:
  ros__parameters:
    history_window: 20          # Max number of past poses to store per track
    cleanup_threshold: 10       # Remove tracks not seen for N cycles
    min_history_length: 3       # Min poses required for velocity estimation
    prediction_steps: 5         # Number of future steps to predict
    prediction_dt: 0.2          # Time between prediction steps (seconds)
```

---

## Troubleshooting

**Q: Detection node publishes but predictor doesn't respond**
- Check topic remapping: `ros2 topic list | grep obstacles`
- Verify predictor is receiving: `ros2 topic echo /obstacles_detected --once`
- Check logs for history buffer initialization

**Q: Visualization markers not showing**
- Verify RViz fixed frame matches `header.frame_id` (usually `map`)
- Check topic name: `ros2 topic echo /cluster_markers --once`
- Enable specific marker namespaces in RViz MarkerArray display

**Q: High CPU usage**
- Reduce costmap resolution or size
- Decrease detection timer frequency (increase period in code)
- Disable visualization node if not needed
- Reduce `prediction_steps` or increase `prediction_dt`
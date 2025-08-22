
# navigation2\_dynamic

Dynamic obstacle **detection → tracking → prediction** for Nav2 (ROS 2).
This repo provides:

* **Foreground mask layer** for Costmap2D (separates dynamic foreground from inflated static background)
* **ODPP**: Obstacle Detection, Persistent Tracking, and Publishing node
* **Predictor service** (CV/KF/VAR stubs) for short-horizon trajectory forecasts
* **Custom messages & service** for obstacle exchange

> Target ROS 2: Humble or newer. Tested with rolling costmaps, TF `map↔odom`, and Nav2.

---

## Repository layout

```
lem-rptu-navigation2_dynamic/
├── foreground_mask_layer/         # costmap2d plugin (C++)
├── nav2_dynamic_msgs/             # msgs + srv
├── obstacle_detection/            # ODPP node (C++)
└── obstacle_predictor/            # predictor service (Python)
```

---

## High-level flow

1. **Foreground mask** isolates dynamic cells from obstacle layer (fast) vs inflated static map (slow).
2. **ClusterEngine** extracts blobs (centroid + convex hull).
3. **TrackerEngine** keeps persistent IDs (Hungarian assignment, gating, history, velocity/heading).
4. **Predictor service** returns `t+N` points per ID (CV by default; KF/VAR placeholders).
5. **ObstacleArray** published with current + predicted positions, kinematics, polygon, covariances.
6. **RViz** markers visualize hulls, centroids, IDs, trajectories, covariance ellipses.

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
  inflation_radius: 20         # cells around static obstacles
  observation_sources: scan    # pass-through to ObstacleLayer
```

### 2) `nav2_dynamic_msgs`

Interfaces shared across nodes.

**Messages (brief)**

* `Obstacle.msg`

  * `int64 id`, `unique_identifier_msgs/UUID uuid`
  * `geometry_msgs/Point[] position`     # position\[0]=current, \[1..N]=predictions
  * `geometry_msgs/Vector3 velocity`, `heading`
  * `geometry_msgs/Polygon polygon`      # convex hull
  * `float64[4] position_covariance`     # row-major 2×2: \[xx, xy, yx, yy]
  * `float64[4] velocity_covariance`

* `ObstacleArray.msg`

  * `std_msgs/Header header`
  * `Obstacle[] obstacles`

* `PredictObstacles.srv`

  * **Request:** `pre_prediction_header`, `uint32 prediction_steps`, `Obstacle[] obstacles_past`
  * **Response:** `post_prediction_header`, `Obstacle[] obstacles_future`

### 3) `obstacle_detection` (C++)

Executable: `dynamic_obstacle_node`

* Brings up a dedicated `Costmap2DROS`
* Clusters blobs (8-connectivity), centroids, convex hull (monotone chain)
* Tracks across ticks (Hungarian assignment, gating radius, missed counts)
* Publishes:

  * `obstacles_array` (`nav2_dynamic_msgs/ObstacleArray`)
  * `cluster_markers` (`visualization_msgs/MarkerArray`)

Asynchronously calls `predict_obstacles` service and **republishes** with predictions when available.

**Important topics**

* **Pub:** `obstacles_array`, `cluster_markers`
* **Sub (via Costmap2DROS/ObstacleLayer):** scans, TF, map, etc.

**Selected params**

```yaml
dynamic_obstacle_node:
  ros__parameters:
    tracking.gate_distance: 1.5
    tracking.max_missed: 3
    prediction.future_len: 5           # N future steps to include in Obstacle.position
    prediction.timeout_ms: 150
    prediction.timer_period_ms: 200
```

### 4) `obstacle_predictor` (Python / rclpy)

Service: `predict_obstacles`

* **Model selection:** `model_type: cv|kf|var`
* Multi-threaded worker pool to process obstacles concurrently
* **CV** (constant velocity) implemented with simple smoothing over recent segments
  **KF/VAR** currently **stubs** delegating to CV with covariance tweaks.

**Selected params**

```yaml
predictor_service_node:
  ros__parameters:
    model_type: cv
    cv:
      history_window: 20
      min_history_length: 3
      velocity_window: 4
      default_dt: 0.5
      min_var: 0.02
    kf:
      default_dt: 0.5
      process_noise: 0.05
      measurement_noise: 0.02
    var:
      order: 2
      max_history: 40
      default_dt: 0.5
```

---

## Build & install

```bash
# create a workspace if needed
mkdir -p ~/ws_nav2dyn/src && cd ~/ws_nav2dyn/src

# clone your repo
git clone -b dev https://github.com/LEM-RPTU/navigation2_dynamic.git lem-rptu-navigation2_dynamic

# build
cd ..
colcon build --symlink-install \
  --packages-select \
    foreground_mask_layer nav2_dynamic_msgs obstacle_detection obstacle_predictor

# source
source install/setup.bash
```

**Dependencies**

* ROS 2 Humble+
* `nav2_costmap_2d`, `rclcpp(_lifecycle)`, `geometry_msgs`, `visualization_msgs`, `unique_identifier_msgs`
* Python node: `rclpy`, `numpy`

---

## Launch

A combined launch starts the ODPP node and predictor service under a namespace:

```bash
ros2 launch obstacle_detection dynamic_obstacle.launch.py \
  namespace:=fleet/skid_steered_two_lidars_0 \
  params_file:=$(ros2 pkg prefix obstacle_detection)/share/obstacle_detection/config/odpp.yaml
```

Expected outputs:

* `/<ns>/dynamic_obstacle_node/obstacles_array`
* `/<ns>/dynamic_obstacle_node/cluster_markers`
* Service `/<ns>/predictor_node/predict_obstacles`

**Frame IDs**

* Costmap uses `map` as `global_frame`; ensure TF `map↔odom` exists.
* Messages carry the costmap’s `global_frame_id`.


## Configuration tips

* **Foreground vs background:** Pick a sufficiently **large `inflation_radius`** for the static map to avoid mislabeling static obstacles as dynamic due to pose drift.
* **Tracking gate:** `tracking.gate_distance` bounds associations; tune for your costmap resolution and dynamics.
* **History length:** The tracker internally caps history (see `TrackerEngine`); the predictor separately windows history.
* **Prediction cadence:** ODPP publishes immediately, then republishes when the async prediction response arrives.



## Visualization (RViz)

Add these displays:

* **MarkerArray** on `cluster_markers` (shows convex hull, centroid sphere, ID text, covariance ellipse, and predicted trajectory line)
* **Path/Points** (optional) for `obstacles_array.obstacles[*].position`
* Costmap layers to confirm foreground masking
---

## Message/service contract (quick view)

```text
Topic: obstacles_array (nav2_dynamic_msgs/ObstacleArray)
  header.stamp, header.frame_id
  obstacles[*]:
    id, uuid
    position[0]        # current centroid (detection or one-cycle prediction)
    position[1..N]     # N predicted steps
    velocity, heading
    polygon            # convex hull
    position_covariance[4], velocity_covariance[4]
```

```text
Service: predict_obstacles (nav2_dynamic_msgs/srv/PredictObstacles)
Request:
  pre_prediction_header.stamp, .frame_id
  prediction_steps: uint32
  obstacles_past: Obstacle[]         # uses .position as history

Response:
  post_prediction_header.stamp, .frame_id
  obstacles_future: Obstacle[]       # .position filled with future points
```

---

## Development notes

* **C++ standards**: plugin uses C++14; ODPP node uses C++17.
* **Assignment**: Hungarian (square cost matrix, distance on centroids), with gating.
* **Timing**: Tracker computes `dt` from steady clock with clamp & fallback; predictor `default_dt` is configurable.
* **Threading**:

  * `Costmap2DROS` spins in its own thread.
  * ODPP timer runs at 500 ms (default); prediction requests are **async**, response updates and republish.
  * Predictor uses a worker queue with multiple threads.

## Known limitations (dev)

* **KF/VAR are placeholders** — CV is the only implemented model; covariances are heuristic.
* **Time sync** between detection tick and prediction response may introduce minor staleness; ODPP republish mitigates it.
* **Licensing mix** across packages (see below).

## Roadmap

* Implement proper **Kalman Filter** (CTRVs/CTRA optional) with tuned Q/R.
* Implement true **VAR(p)** multi-step with covariance propagation.
* Unit/integration tests (CI).
* Expand RViz helpers (poses, arrows).
* Example bag & Gazebo simulation world.

---

## Contributing

PRs and issues welcome. Please follow ROS 2 style guides, keep functions small and tested, and document new params in `odpp.yaml` or package READMEs.

## License

* **Top-level**: Apache-2.0 (see `LICENSE`).
* Packages may declare **BSD-3-Clause** or Apache-2.0 in their `package.xml`.
  Always check the package’s own `package.xml` for the definitive license.


## Acknowledgments

Based on Nav2 and ROS 2 ecosystems. Thanks to the contributors and advisors involved in design discussions and review.

## Contact

- Maintainer: **[Riyan Cyriac Jose](https://github.com/joseriyancyriac)**


## Appendix: Minimal YAML snippet

```yaml
/**/dynamic_obstacle_costmap:
  dynamic_obstacle_costmap:
    ros__parameters:
      global_frame: map
      robot_base_frame: base_footprint
      rolling_window: true
      width: 50
      height: 50
      resolution: 0.1
      plugins: ["foreground_mask_layer", "denoise_layer", "inflation_layer"]
      foreground_mask_layer:
        plugin: "foreground_mask::ForegroundMaskLayer"
        map_topic: "/fleet/skid_steered_two_lidars_0/map"
        inflation_radius: 20
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        inflation_radius: 1.0

/**/dynamic_obstacle_node:
  ros__parameters:
    tracking.gate_distance: 1.5
    tracking.max_missed: 3
    prediction.future_len: 5
    prediction.timeout_ms: 150

/**/predictor_service_node:
  ros__parameters:
    model_type: cv
    cv.default_dt: 0.5
```
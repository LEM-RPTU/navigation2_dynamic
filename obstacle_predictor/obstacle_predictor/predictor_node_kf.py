"""
Predictor node: Kalman Filter-based prediction for dynamic obstacles.
Subscribes to detections, publishes predictions for Nav2 consumption.
"""

import math
import threading
from typing import List, Dict

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance
from nav2_dynamic_interface.msg import Obstacle, ObstacleArray


class PredictorNode(Node):
    def __init__(self):
        super().__init__('predictor_node')

        # Parameters
        self.declare_parameter('history_window', 10)        # Max history length per ID
        self.declare_parameter('cleanup_threshold', 5)      # Remove tracks after N cycles without update
        self.declare_parameter('prediction_steps', 5)       # Number of future steps
        self.declare_parameter('prediction_dt', 0.2)        # Prediction timestep (seconds)
        
        # KF-specific parameters
        self.declare_parameter('kf.process_noise_pos', 0.1)      # Process noise for position
        self.declare_parameter('kf.process_noise_vel', 0.5)      # Process noise for velocity
        self.declare_parameter('kf.measurement_noise', 0.2)      # Measurement noise

        # History storage: id -> list of PoseWithCovariance
        self.history: Dict[int, List[PoseWithCovariance]] = {}
        self.last_seen: Dict[int, int] = {}  # Track cycles since last update
        self.cycle_count = 0
        self.history_lock = threading.Lock()
        
        # KF state storage: id -> {'x': state_vector, 'P': covariance_matrix}
        # State vector: [px, py, vx, vy]
        # Covariance: 4x4 matrix
        self.kf_states: Dict[int, Dict] = {}

        # Create callback groups for concurrency
        self.sub_group = ReentrantCallbackGroup()
        self.pub_group = ReentrantCallbackGroup()

        # Subscriber: current detections from detection_node
        self.detections_sub = self.create_subscription(
            ObstacleArray,
            'obstacles_detected',
            self.on_detections,
            10,
            callback_group=self.sub_group
        )

        # Publisher: predictions for Nav2 and visualization
        self.predictions_pub = self.create_publisher(
            ObstacleArray,
            'obstacles_array',
            10,
            callback_group=self.pub_group
        )

        self.get_logger().info('PredictorNode initialized (Kalman Filter model)')

    def on_detections(self, msg: ObstacleArray):
        """Process incoming detections, update history, compute and publish predictions"""
        
        # Get parameters
        history_window = self.get_parameter('history_window').get_parameter_value().integer_value
        cleanup_threshold = self.get_parameter('cleanup_threshold').get_parameter_value().integer_value
        prediction_steps = self.get_parameter('prediction_steps').get_parameter_value().integer_value
        prediction_dt = self.get_parameter('prediction_dt').get_parameter_value().double_value

        # Track which IDs are in this detection cycle
        current_detection_ids = set()

        # Update history
        with self.history_lock:
            self.cycle_count += 1

            for detection in msg.obstacles:
                obs_id = detection.id
                current_detection_ids.add(obs_id)

                # Initialize history if new ID
                if obs_id not in self.history:
                    self.history[obs_id] = []
                    self.get_logger().debug(f"New track ID {obs_id}")

                # Append current pose (detection.position[0] is current)
                if detection.position:
                    self.history[obs_id].append(detection.position[0])
                    # Cap history
                    if len(self.history[obs_id]) > history_window:
                        self.history[obs_id] = self.history[obs_id][-history_window:]

                # Track last seen
                self.last_seen[obs_id] = self.cycle_count

            # This handles obstacles leaving the costmap window
            missing_ids = set(self.history.keys()) - current_detection_ids
            for obs_id in missing_ids:
                # Only remove if it's been missing for cleanup_threshold cycles
                if self.cycle_count - self.last_seen.get(obs_id, 0) > cleanup_threshold:
                    self.get_logger().debug(f"Cleaning up stale ID {obs_id}")
                    del self.history[obs_id]
                    del self.last_seen[obs_id]

        # Build prediction message ONLY for obstacles in current detections
        prediction_msg = ObstacleArray()
        prediction_msg.header = msg.header

        for detection in msg.obstacles:
            obs_id = detection.id

            # Get history for this ID (thread-safe copy)
            with self.history_lock:
                hist = list(self.history.get(obs_id, []))

            # Compute predictions
            future_poses, vel_twists = self._predict_cv(hist, prediction_steps, prediction_dt)

            # Build obstacle message
            predicted_ob = Obstacle()
            predicted_ob.header = detection.header
            predicted_ob.id = detection.id
            predicted_ob.uuid = detection.uuid
            predicted_ob.polygon = detection.polygon

            # Set delta_time
            predicted_ob.delta_time.sec = int(prediction_dt)
            predicted_ob.delta_time.nanosec = int((prediction_dt - int(prediction_dt)) * 1e9)

            # Position: [current, future_1, future_2, ...]
            predicted_ob.position = [detection.position[0]] + future_poses
            predicted_ob.velocity = vel_twists

            prediction_msg.obstacles.append(predicted_ob)

        # Publish predictions
        self.predictions_pub.publish(prediction_msg)
        self.get_logger().debug(f"Published predictions for {len(prediction_msg.obstacles)} obstacles")

    def _predict_kf(
        self,
        obs_id: int,
        history: List[PoseWithCovariance],
        steps: int,
        dt: float
    ) -> tuple[List[PoseWithCovariance], List[TwistWithCovariance]]:
        """
        Kalman Filter prediction using constant velocity motion model.
        
        State vector: [px, py, vx, vy]
        Motion model: x_{k+1} = F * x_k + w, where F is state transition matrix
        
        Args:
            obs_id: Obstacle ID for state tracking
            history: List of past poses (newest last)
            steps: Number of future steps to predict
            dt: Time step between predictions (seconds)

        Returns:
            Tuple of (future_poses, velocities)
        """
        # TODO: Implement Kalman Filter prediction
        # 
        # Suggested implementation steps:
        # 1. Initialize or retrieve KF state for obs_id
        #    - State: [px, py, vx, vy]
        #    - Covariance: 4x4 matrix P
        # 
        # 2. Update step (if new measurement available):
        #    - z = [px, py] from history[-1]
        #    - H = [[1,0,0,0], [0,1,0,0]]  # Measurement matrix
        #    - R = measurement_noise * I_2x2
        #    - K = P @ H^T @ (H @ P @ H^T + R)^-1  # Kalman gain
        #    - x = x + K @ (z - H @ x)
        #    - P = (I - K @ H) @ P
        # 
        # 3. Prediction step (for k steps):
        #    - F = [[1,0,dt,0], [0,1,0,dt], [0,0,1,0], [0,0,0,1]]
        #    - Q = process_noise * [[dt^3/3, 0, dt^2/2, 0],
        #                           [0, dt^3/3, 0, dt^2/2],
        #                           [dt^2/2, 0, dt, 0],
        #                           [0, dt^2/2, 0, dt]]
        #    - x = F @ x
        #    - P = F @ P @ F^T + Q
        # 
        # 4. Store predictions in PoseWithCovariance and TwistWithCovariance
        
        self.get_logger().warn("KF prediction not implemented - using fallback")

        # Return empty lists (no predictions)
        return ([], [])
    
def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor(num_threads=4)
    node = PredictorNode()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
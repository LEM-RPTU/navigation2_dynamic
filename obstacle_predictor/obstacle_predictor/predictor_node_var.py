"""
Predictor node: Vector Autoregression (VAR) prediction for dynamic obstacles.
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
        
        # VAR-specific parameters
        self.declare_parameter('var.lag_order', 3)          # Number of lags (p in VAR(p))
        self.declare_parameter('var.min_samples', 10)       # Min history for VAR estimation
        self.declare_parameter('var.regularization', 0.01)  # L2 regularization coefficient

        # History storage: id -> list of PoseWithCovariance
        self.history: Dict[int, List[PoseWithCovariance]] = {}
        self.last_seen: Dict[int, int] = {}  # Track cycles since last update
        self.cycle_count = 0
        self.history_lock = threading.Lock()
        
        # VAR coefficient storage: id -> {'A': [A1, A2, ..., Ap], 'Sigma': residual_covariance}
        # A_i are 2x2 coefficient matrices for [x, y] at lag i
        self.var_models: Dict[int, Dict] = {}

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

        self.get_logger().info('PredictorNode initialized (Vector Autoregression model)')

    def on_detections(self, msg: ObstacleArray):
        """Process incoming detections, update history, compute and publish predictions"""
        if not msg.obstacles:
            return

        # Get parameters
        history_window = self.get_parameter('history_window').get_parameter_value().integer_value
        cleanup_threshold = self.get_parameter('cleanup_threshold').get_parameter_value().integer_value
        prediction_steps = self.get_parameter('prediction_steps').get_parameter_value().integer_value
        prediction_dt = self.get_parameter('prediction_dt').get_parameter_value().double_value

        # Update history
        with self.history_lock:
            self.cycle_count += 1

            for detection in msg.obstacles:
                obs_id = detection.id

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

            # Cleanup stale IDs
            stale_ids = [
                obs_id for obs_id, last_cycle in self.last_seen.items()
                if self.cycle_count - last_cycle > cleanup_threshold
            ]
            for obs_id in stale_ids:
                self.get_logger().debug(f"Cleaning up stale ID {obs_id}")
                del self.history[obs_id]
                del self.last_seen[obs_id]
                del self.var_models[obs_id]

        # Build prediction message
        prediction_msg = ObstacleArray()
        prediction_msg.header = msg.header  # Preserve timestamp and frame

        for detection in msg.obstacles:
            obs_id = detection.id

            # Get history for this ID (thread-safe copy)
            with self.history_lock:
                hist = list(self.history.get(obs_id, []))

            # Compute predictions using VAR
            future_poses, vel_twists = self._predict_var(obs_id, hist, prediction_steps, prediction_dt)

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

    def _predict_var(
        self,
        obs_id: int,
        history: List[PoseWithCovariance],
        steps: int,
        dt: float
    ) -> tuple[List[PoseWithCovariance], List[TwistWithCovariance]]:
        """
        Vector Autoregression prediction for obstacle trajectory.
        
        VAR(p) model: y_t = A_1 * y_{t-1} + A_2 * y_{t-2} + ... + A_p * y_{t-p} + e_t
        where y_t = [x_t, y_t]^T (position vector)
        
        Args:
            obs_id: Obstacle ID for model tracking
            history: List of past poses (newest last)
            steps: Number of future steps to predict
            dt: Time step between predictions (seconds)

        Returns:
            Tuple of (future_poses, velocities)
        """
        # TODO: Implement VAR prediction
        # 
        # Suggested implementation steps:
        # 1. Extract position time series from history:
        #    - Y = [[x_0, y_0], [x_1, y_1], ..., [x_n, y_n]]
        # 
        # 2. Fit VAR(p) model using OLS (Ordinary Least Squares):
        #    - Build lagged design matrix X (n-p samples x 2p features)
        #    - Target matrix Y_target = Y[p:, :]
        #    - Estimate coefficients: A = (X^T X + lambda*I)^-1 X^T Y_target
        #    - Residual covariance: Sigma = cov(Y_target - X @ A)
        # 
        # 3. Multi-step ahead prediction:
        #    - Initialize: y_pred = [y_{n-p+1}, ..., y_n]  (last p observations)
        #    - For k = 1 to steps:
        #        y_{n+k} = A_1 @ y_{n+k-1} + A_2 @ y_{n+k-2} + ... + A_p @ y_{n+k-p}
        #        Append y_{n+k} to predictions
        #        Update sliding window: drop oldest, add newest
        # 
        # 4. Compute velocities from position differences:
        #    - v_k = (y_{k+1} - y_k) / dt
        # 
        # 5. Covariance propagation (optional):
        #    - Use forecast error variance recursion
        #    - Cov(e_{k+1}) = A @ Cov(e_k) @ A^T + Sigma
        # 
        # 6. Store in PoseWithCovariance and TwistWithCovariance
        
        self.get_logger().warn("VAR prediction not implemented - using fallback")

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
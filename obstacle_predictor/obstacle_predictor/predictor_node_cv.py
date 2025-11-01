"""
Predictor node: maintains history per obstacle ID, predicts using constant velocity model.
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
        self.declare_parameter('cleanup_threshold', 5)     # remove tracks after N cycles without update
        self.declare_parameter('prediction_steps', 5)       # Number of future steps
        self.declare_parameter('prediction_dt', 0.2)        # Prediction timestep (seconds)

        # History storage: id -> list of PoseWithCovariance
        self.history: Dict[int, List[PoseWithCovariance]] = {}
        self.last_seen: Dict[int, int] = {}  # Track cycles since last update
        self.cycle_count = 0
        self.history_lock = threading.Lock()

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

        self.get_logger().info('PredictorNode initialized (constant velocity model)')

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

        # Build prediction message
        prediction_msg = ObstacleArray()
        prediction_msg.header = msg.header  # Preserve timestamp and frame

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

    def _predict_cv(
        self,
        history: List[PoseWithCovariance],
        steps: int,
        dt: float
    ) -> tuple[List[PoseWithCovariance], List[TwistWithCovariance]]:
        """
        Constant velocity prediction using stored history.

        Args:
            history: List of past poses (newest last)
            steps: Number of future steps to predict
            dt: Time step between predictions (seconds)

        Returns:
            Tuple of (future_poses, velocities) where:
            - future_poses[k] = predicted pose at t + dt*(k+1)
            - velocities[k] = estimated velocity at future step k
        """

        if not history:
            zero_pose = PoseWithCovariance()
            zero_pose.pose.position.x = 0.0
            zero_pose.pose.position.y = 0.0
            zero_pose.pose.position.z = 0.0
            zero_pose.pose.orientation.w = 1.0
            zero_pose.covariance = [
                0.05, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.05, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.01
            ]

            zero_twist = TwistWithCovariance()
            zero_twist.twist.linear.x = 0.0
            zero_twist.twist.linear.y = 0.0
            zero_twist.twist.linear.z = 0.0
            zero_twist.covariance = zero_pose.covariance

            future_poses = [zero_pose for _ in range(steps)]
            vel_twists = [zero_twist for _ in range(steps)]
            return future_poses, vel_twists

        n = len(history)
        points = [pose.pose.position for pose in history]

        # Estimate velocity: either last two points OR entire history
        vx = vy = 0.0
        if n >= 2:
            # Use entire history (first to last point)
            dx = points[-1].x - points[0].x
            dy = points[-1].y - points[0].y
            time_span = (n - 1) * dt
            vx = dx / time_span
            vy = dy / time_span

        # Require minimum history for velocity estimation
        if n < 2:
            vx = vy = 0.0

        last_point = points[-1]
        future_poses: List[PoseWithCovariance] = []
        vel_twists: List[TwistWithCovariance] = []

        # Compute covariance from history scatter
        if n >= 2:
            mean_x = sum(p.x for p in points) / n
            mean_y = sum(p.y for p in points) / n
            var_x = sum((p.x - mean_x) ** 2 for p in points) / (n - 1)
            var_y = sum((p.y - mean_y) ** 2 for p in points) / (n - 1)
        else:
            var_x = var_y = 0.01

        for k in range(1, steps + 1):
            # Pose prediction
            pose = PoseWithCovariance()
            pose.pose.position.x = last_point.x + vx * dt * k
            pose.pose.position.y = last_point.y + vy * dt * k
            pose.pose.position.z = last_point.z
            pose.pose.orientation.w = 1.0

            # Covariance (grows with prediction horizon)
            uncertainty_scale = 1.0 + 0.1 * k  # Linear growth
            pose.covariance = [
                var_x * uncertainty_scale, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, var_y * uncertainty_scale, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.01
            ]
            future_poses.append(pose)

            # Velocity (constant velocity assumption)
            twist = TwistWithCovariance()
            twist.twist.linear.x = vx
            twist.twist.linear.y = vy
            twist.twist.linear.z = 0.0
            twist.twist.angular.z = 0.0

            # Velocity uncertainty
            vel_var_x = (abs(vx) + 1e-3) * 0.1
            vel_var_y = (abs(vy) + 1e-3) * 0.1
            twist.covariance = [
                vel_var_x, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, vel_var_y, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.01
            ]
            vel_twists.append(twist)

        return future_poses, vel_twists


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
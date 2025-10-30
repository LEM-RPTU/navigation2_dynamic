#!/usr/bin/env python3
# Predictor node: stateful, maintains history per obstacle ID, matches new Obstacle.msg format

import math
import time
import threading
from queue import Queue, Empty
from typing import List, Tuple, Dict

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Point, PoseWithCovariance, TwistWithCovariance
from std_msgs.msg import Header
from builtin_interfaces.msg import Duration
from nav2_dynamic_interface.msg import Obstacle
from nav2_dynamic_interface.srv import PredictObstacles

class PredictorServiceNode(Node):
    def __init__(self):
        super().__init__('predictor_service_node')

        # Generic selector
        self.declare_parameter('model_type', 'cv')  # cv | kf | var

        # History management
        self.declare_parameter('history_window', 20)  # Max history length per ID
        self.declare_parameter('cleanup_threshold', 10)  # Cleanup after N cycles without update

        # CV model parameters
        self.declare_parameter('cv.min_history_length', 3)
        self.declare_parameter('cv.velocity_window', 4)

        # KF model (stub) parameters
        self.declare_parameter('kf.process_noise', 0.05)
        self.declare_parameter('kf.measurement_noise', 0.02)

        # VAR model (stub) parameters
        self.declare_parameter('var.order', 2)

        # History storage: id -> list of PoseWithCovariance
        self.history: Dict[int, List[PoseWithCovariance]] = {}
        self.last_seen: Dict[int, int] = {}  # Track cycles since last update
        self.cycle_count = 0
        self.history_lock = threading.Lock()

        # Create callback groups for concurrency
        self.service_group = ReentrantCallbackGroup()
        
        # Service with threading
        self.prediction_queue = Queue(maxsize=10)
        self.worker_threads = []
        for i in range(4):
            thread = threading.Thread(target=self._prediction_worker, daemon=True)
            thread.start()
            self.worker_threads.append(thread)
        
        self.srv = self.create_service(
            PredictObstacles,
            'predict_obstacles',
            self.handle_predict,
            callback_group=self.service_group
        )
        
        self.get_logger().info('PredictorServiceNode ready (stateful, history per ID)')

    def handle_predict(self, request, response):
        """Queue prediction request for worker threads, update history"""
        start_time = time.time()
        self.get_logger().info(f"Received prediction request with {len(request.obstacles_past)} obstacles")
        
        response.post_prediction_header = Header()
        response.post_prediction_header.stamp = self.get_clock().now().to_msg()
        if request.pre_prediction_header.frame_id:
            response.post_prediction_header.frame_id = request.pre_prediction_header.frame_id
        
        steps = request.prediction_steps
        if steps <= 0:
            return response
        
        # Update history for incoming obstacles
        hw = self.get_parameter('history_window').get_parameter_value().integer_value
        with self.history_lock:
            self.cycle_count += 1
            for past_ob in request.obstacles_past:
                obs_id = past_ob.id
                
                # Initialize history if new ID
                if obs_id not in self.history:
                    self.history[obs_id] = []
                
                # Append current pose (assume position[0] is current)
                if past_ob.position:
                    self.history[obs_id].append(past_ob.position[0])
                    # Cap history
                    if len(self.history[obs_id]) > hw:
                        self.history[obs_id] = self.history[obs_id][-hw:]
                
                # Track last seen
                self.last_seen[obs_id] = self.cycle_count
            
            # Cleanup stale IDs
            cleanup_thresh = self.get_parameter('cleanup_threshold').get_parameter_value().integer_value
            stale_ids = [
                obs_id for obs_id, last_cycle in self.last_seen.items()
                if self.cycle_count - last_cycle > cleanup_thresh
            ]
            for obs_id in stale_ids:
                self.get_logger().info(f"Cleaning up stale ID {obs_id}")
                del self.history[obs_id]
                del self.last_seen[obs_id]
        
        # Process each obstacle in parallel
        result_queue = Queue()
        tasks_submitted = 0 
        
        for past_ob in request.obstacles_past:
            self.prediction_queue.put((past_ob, steps, result_queue))
            tasks_submitted += 1
            
        # Wait for all results
        for _ in range(tasks_submitted):
            future_ob = result_queue.get()
            response.obstacles_future.append(future_ob)
        
        elapsed = time.time() - start_time
        self.get_logger().info(f"Prediction completed in {elapsed:.4f} seconds")
        return response
    
    def _prediction_worker(self):
        """Worker thread that processes predictions from the queue"""
        while rclpy.ok():
            try:
                past_ob, steps, result_queue = self.prediction_queue.get(timeout=0.1)
                
                future_ob = Obstacle()
                future_ob.id = past_ob.id
                future_ob.uuid = past_ob.uuid
                future_ob.header = past_ob.header
                future_ob.delta_time = past_ob.delta_time
                future_ob.polygon = past_ob.polygon
                
                # Get history for this ID (thread-safe copy)
                with self.history_lock:
                    hist = list(self.history.get(past_ob.id, []))
                
                # Get model type
                model_type = self.get_parameter('model_type').get_parameter_value().string_value.lower()
                
                # Compute dt from delta_time
                dt_sec = past_ob.delta_time.sec + past_ob.delta_time.nanosec / 1e9
                if dt_sec <= 0:
                    dt_sec = 0.2  # Fallback
                
                # Compute prediction
                if model_type == 'cv':
                    future_poses, vel_twists = self._predict_cv(hist, steps, dt_sec)
                elif model_type == 'kf':
                    future_poses, vel_twists = self._predict_kf_stub(hist, steps, dt_sec)
                elif model_type == 'var':
                    future_poses, vel_twists = self._predict_var_stub(hist, steps, dt_sec)
                else:
                    future_poses, vel_twists = self._predict_cv(hist, steps, dt_sec)
                
                # Fill response: position = future_poses (predictions only, current already in request)
                future_ob.position = future_poses
                future_ob.velocity = vel_twists
                
                result_queue.put(future_ob)
                self.prediction_queue.task_done()
                
            except Empty:
                continue
            except Exception as e:
                self.get_logger().error(f"Error in prediction worker: {str(e)}")

    def _predict_cv(self, history: List[PoseWithCovariance], steps: int, dt: float) -> Tuple[List[PoseWithCovariance], List[TwistWithCovariance]]:
        """Constant velocity prediction using stored history"""
        min_hist = self.get_parameter('cv.min_history_length').get_parameter_value().integer_value
        vel_w = max(1, self.get_parameter('cv.velocity_window').get_parameter_value().integer_value)

        if not history:
            return self._repeat_static(steps, dt)

        n = len(history)
        points = [pose.pose.position for pose in history]

        # Estimate velocity
        vx = vy = 0.0
        if n >= 2:
            seg_count = min(vel_w, n - 1)
            dx_sum = dy_sum = 0.0
            for i in range(n - seg_count, n):
                dx_sum += (points[i].x - points[i - 1].x)
                dy_sum += (points[i].y - points[i - 1].y)
            vx = dx_sum / (seg_count * dt)
            vy = dy_sum / (seg_count * dt)

        if n < min_hist:
            vx = vy = 0.0

        last_point = points[-1]
        future_poses: List[PoseWithCovariance] = []
        vel_twists: List[TwistWithCovariance] = []
        
        for k in range(1, steps + 1):
            # Pose
            pose = PoseWithCovariance()
            pose.pose.position.x = last_point.x + vx * dt * k
            pose.pose.position.y = last_point.y + vy * dt * k
            pose.pose.position.z = last_point.z
            pose.pose.orientation.w = 1.0
            
            # Compute covariance from history scatter
            if n >= 2:
                mean_x = sum(p.x for p in points) / n
                mean_y = sum(p.y for p in points) / n
                var_x = sum((p.x - mean_x) ** 2 for p in points) / (n - 1)
                var_y = sum((p.y - mean_y) ** 2 for p in points) / (n - 1)
            else:
                var_x = var_y = 0.01
            
            # 6x6 covariance (row-major: xx, xy, xz, ..., yaw)
            pose.covariance = [
                var_x, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, var_y, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.01
            ]
            future_poses.append(pose)
            
            # Twist
            twist = TwistWithCovariance()
            twist.twist.linear.x = vx
            twist.twist.linear.y = vy
            twist.twist.linear.z = 0.0
            twist.twist.angular.z = 0.0
            
            vel_var_x = (abs(vx) + 1e-3) * 0.01
            vel_var_y = (abs(vy) + 1e-3) * 0.01
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

    def _predict_kf_stub(self, history: List[PoseWithCovariance], steps: int, dt: float):
        """KF stub: defer to CV with inflated covariance"""
        future_poses, vel_twists = self._predict_cv(history, steps, dt)
        for pose in future_poses:
            pose.covariance = [c * 2.0 for c in pose.covariance]
        for twist in vel_twists:
            twist.covariance = [c * 2.0 for c in twist.covariance]
        return future_poses, vel_twists

    def _predict_var_stub(self, history: List[PoseWithCovariance], steps: int, dt: float):
        """VAR stub: defer to CV with different inflation"""
        future_poses, vel_twists = self._predict_cv(history, steps, dt)
        for pose in future_poses:
            pose.covariance[0] *= 1.5  # xx
            pose.covariance[7] *= 1.5  # yy
        return future_poses, vel_twists

    def _repeat_static(self, steps: int, dt: float):
        """Fallback for no history: stationary predictions"""
        zero_pose = PoseWithCovariance()
        zero_pose.pose.position = Point()
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
        zero_twist.covariance = zero_pose.covariance
        
        future_poses = [zero_pose for _ in range(steps)]
        vel_twists = [zero_twist for _ in range(steps)]
        return future_poses, vel_twists


def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor(num_threads=6)
    node = PredictorServiceNode()
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
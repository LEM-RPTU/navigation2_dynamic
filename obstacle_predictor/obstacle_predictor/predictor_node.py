#!/usr/bin/env python3
# Predictor node: stateless, single-request-per-obstacle, one-cycle-lag friendly

import math
import time
import threading
from queue import Queue, Empty
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import Header
from nav2_dynamic_interface.msg import Obstacle
from nav2_dynamic_interface.srv import PredictObstacles

class PredictorServiceNode(Node):
    def __init__(self):
        super().__init__('predictor_service_node')

        # Generic selector
        self.declare_parameter('model_type', 'cv')  # cv | kf | var

        # CV model parameters
        self.declare_parameter('cv.history_window', 20)
        self.declare_parameter('cv.min_history_length', 3)
        self.declare_parameter('cv.velocity_window', 4)
        self.declare_parameter('cv.default_dt', 0.5)
        self.declare_parameter('cv.confidence_base', 0.85)  # (Not used yet)

        # KF model (stub) parameters
        self.declare_parameter('kf.process_noise', 0.05)
        self.declare_parameter('kf.measurement_noise', 0.02)
        self.declare_parameter('kf.init_velocity_var', 0.25)
        self.declare_parameter('kf.default_dt', 0.5)
        self.declare_parameter('kf.history_window', 30)

        # VAR model (stub) parameters
        self.declare_parameter('var.order', 2)
        self.declare_parameter('var.max_history', 40)
        self.declare_parameter('var.default_dt', 0.5)

        # Create callback groups for concurrency
        self.service_group = ReentrantCallbackGroup()
        
        # Service with threading
        self.prediction_queue = Queue(maxsize=5)  # Max 5 pending predictions
        self.worker_threads = []
        for i in range(4):  # Use 4 worker threads
            thread = threading.Thread(target=self._prediction_worker, daemon=True)
            thread.start()
            self.worker_threads.append(thread)
        
        # Service in reentrant group to allow concurrent processing
        self.srv = self.create_service(
            PredictObstacles,
            'predict_obstacles',
            self.handle_predict,
            callback_group=self.service_group
        )
        
        self.get_logger().info('PredictorServiceNode ready with multi-threading')

    # ---------------- Service Callback ----------------
    def handle_predict(self, request, response):
        """
        Queue prediction request for worker threads instead of blocking
        """
        start_time = time.time()
        self.get_logger().info(f"Received prediction request with {len(request.obstacles_past)} obstacles")
        
        # Fill out basic response header
        response.post_prediction_header = Header()
        response.post_prediction_header.stamp = self.get_clock().now().to_msg()
        if request.pre_prediction_header.frame_id:
            response.post_prediction_header.frame_id = request.pre_prediction_header.frame_id
        
        steps = request.prediction_steps
        if steps <= 0:
            return response
        
        # Process each obstacle in parallel
        result_queue = Queue()
        tasks_submitted = 0 
        
        for past_ob in request.obstacles_past:
            # Queue prediction task
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
                
                # Create obstacle for response
                future_ob = Obstacle()
                future_ob.id = past_ob.id
                future_ob.uuid = past_ob.uuid
                
                # Get model type
                model_type = self.get_parameter('model_type').get_parameter_value().string_value.lower()
                
                # Compute prediction based on model
                if model_type == 'cv':
                    future_points, vel, heading, pos_cov, vel_cov = self._predict_cv(past_ob.position, steps)
                elif model_type == 'kf':
                    future_points, vel, heading, pos_cov, vel_cov = self._predict_kf_stub(past_ob.position, steps)
                elif model_type == 'var':
                    future_points, vel, heading, pos_cov, vel_cov = self._predict_var_stub(past_ob.position, steps)
                else:
                    future_points, vel, heading, pos_cov, vel_cov = self._predict_cv(past_ob.position, steps)
                
                # Fill response
                future_ob.position = future_points
                future_ob.velocity = vel
                future_ob.heading = heading
                
                # Ensure covariance is properly formatted (exactly 4 elements each)
                if len(pos_cov) != 4:
                    self.get_logger().warn(f"Position covariance wrong size {len(pos_cov)}, fixing")
                    pos_cov = [0.05, 0.0, 0.0, 0.05]
                if len(vel_cov) != 4:
                    self.get_logger().warn(f"Velocity covariance wrong size {len(vel_cov)}, fixing")
                    vel_cov = [0.01, 0.0, 0.0, 0.01]
                
                future_ob.position_covariance = pos_cov
                future_ob.velocity_covariance = vel_cov
                
                # Put result in result queue
                result_queue.put(future_ob)
                self.prediction_queue.task_done()
                
            except Empty:
                # No work to do
                continue
            except Exception as e:
                self.get_logger().error(f"Error in prediction worker: {str(e)}")

    # ---------------- Constant Velocity Implementation ----------------
    def _predict_cv(self, history: List[Point], steps: int) -> Tuple[List[Point], Vector3, Vector3, List[float], List[float]]:
        hw = self.get_parameter('cv.history_window').get_parameter_value().integer_value
        min_hist = self.get_parameter('cv.min_history_length').get_parameter_value().integer_value
        vel_w = max(1, self.get_parameter('cv.velocity_window').get_parameter_value().integer_value)
        dt = float(self.get_parameter('cv.default_dt').get_parameter_value().double_value)

        if not history:
            return self._repeat_origin(steps)

        hist = history[-hw:] if hw > 0 else history
        n = len(hist)

        # Default: stationary
        vx = vy = 0.0
        if n >= 2:
            # Use last K segments for smoothing
            seg_count = min(vel_w, n - 1)
            dx_sum = 0.0
            dy_sum = 0.0
            for i in range(n - seg_count, n - 1):
                dx_sum += (hist[i + 1].x - hist[i].x)
                dy_sum += (hist[i + 1].y - hist[i].y)
            vx = dx_sum / (seg_count * dt)
            vy = dy_sum / (seg_count * dt)

        # If insufficient history, treat as stationary (or very low confidence – not encoded here)
        if n < min_hist:
            vx = vy = 0.0

        speed = math.hypot(vx, vy)
        heading = Vector3()
        if speed > 1e-6:
            heading.x = vx / speed
            heading.y = vy / speed
        else:
            heading.x = 0.0
            heading.y = 0.0
        heading.z = 0.0

        vel_vec = Vector3()
        vel_vec.x = vx
        vel_vec.y = vy
        vel_vec.z = 0.0

        last = hist[-1]
        preds: List[Point] = []
        for k in range(1, steps + 1):
            p = Point()
            p.x = last.x + vx * dt * k
            p.y = last.y + vy * dt * k
            p.z = last.z
            preds.append(p)

        # Simple sample variance for position (historical scatter)
        if n >= 2:
            mean_x = sum(p.x for p in hist) / n
            mean_y = sum(p.y for p in hist) / n
            var_x = sum((p.x - mean_x) ** 2 for p in hist) / (n - 1)
            var_y = sum((p.y - mean_y) ** 2 for p in hist) / (n - 1)
        else:
            var_x = var_y = 0.01

        # Very naive velocity variance proxy
        vel_var_x = (abs(vx) + 1e-3) * 0.01
        vel_var_y = (abs(vy) + 1e-3) * 0.01

        pos_cov = [var_x, 0.0, 0.0, var_y]
        vel_cov = [vel_var_x, 0.0, 0.0, vel_var_y]

        return preds, vel_vec, heading, pos_cov, vel_cov

    # ---------------- KF Stub ----------------
    def _predict_kf_stub(self, history: List[Point], steps: int):
        # Placeholder: defer to CV until KF implemented
        preds, vel, heading, pos_cov, vel_cov = self._predict_cv(history, steps)
        # Inflate covariance to signal lower trust vs proper KF
        pos_cov = [c * 2.0 for c in pos_cov]
        vel_cov = [c * 2.0 for c in vel_cov]
        return preds, vel, heading, pos_cov, vel_cov

    # ---------------- VAR Stub ----------------
    def _predict_var_stub(self, history: List[Point], steps: int):
        # Placeholder: copy CV prediction (later replace with true VAR multi-step)
        preds, vel, heading, pos_cov, vel_cov = self._predict_cv(history, steps)
        # Slightly inflate different axis to simulate different model
        pos_cov = [pos_cov[0] * 1.5, 0.0, 0.0, pos_cov[3] * 1.5]
        return preds, vel, heading, pos_cov, vel_cov

    # ---------------- Helpers ----------------
    def _repeat_origin(self, steps: int):
        zero = Point()
        preds = [zero for _ in range(steps)]
        vel = Vector3()
        heading = Vector3()
        pos_cov = [0.05, 0.0, 0.0, 0.05]
        vel_cov = [0.05, 0.0, 0.0, 0.05]
        return preds, vel, heading, pos_cov, vel_cov


def main(args=None):
    rclpy.init(args=args)
    
    # Create node with multithreaded executor
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
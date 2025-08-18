#!/usr/bin/env python3
# Predictor node: stateless, single-request-per-obstacle, one-cycle-lag friendly

import rclpy
from rclpy.node import Node
import numpy as np
from typing import List
from nav2_dynamic_msgs.msg import PredictionRequest, PredictionResponse
from geometry_msgs.msg import Point

class PredictorNode(Node):
    def __init__(self):
        super().__init__('predictor_node')

        # Parameters: core timing and history behavior
        self.declare_parameter('var_time_steps', 0.5)           # Fallback dt if request lacks it
        self.declare_parameter('min_history_length', 3)         # Minimum samples to estimate velocity
        self.declare_parameter('history_window', 20)            # Only use last-N points for computation
        self.declare_parameter('velocity_window', 3)            # Smooth velocity over last-K segments
        self.declare_parameter('confidence_base', 0.85)         # Base confidence for well-formed inputs

        self.var_time_steps = float(self.get_parameter('var_time_steps').value)
        self.min_history_length = int(self.get_parameter('min_history_length').value)
        self.history_window = int(self.get_parameter('history_window').value)
        self.velocity_window = max(1, int(self.get_parameter('velocity_window').value))
        self.confidence_base = float(self.get_parameter('confidence_base').value)

        # Subscribers and Publishers
        self.prediction_request_sub = self.create_subscription(
            PredictionRequest,
            'prediction_request',
            self.prediction_request_callback,
            10
        )

        self.prediction_response_pub = self.create_publisher(
            PredictionResponse,
            'prediction_response',
            10
        )

        self.get_logger().info('PredictorNode started.')
    
    def prediction_request_callback(self, msg: PredictionRequest):
        """Handle prediction requests from tracker (stateless, async-safe)."""
        response = PredictionResponse()

        # Basic passthrough fields
        response.header = msg.header
        if hasattr(response, 'obstacle_id') and hasattr(msg, 'obstacle_id'):
            response.obstacle_id = msg.obstacle_id

        # Optional correlation fields (echo back if present)
        for fld in ('req_seq', 'uuid'):
            if hasattr(msg, fld) and hasattr(response, fld):
                setattr(response, fld, getattr(msg, fld))

        # Use only the last-N history points to keep computation/message light
        history: List[Point] = list(msg.history)[-self.history_window:] if hasattr(msg, 'history') else []

        # Determine dt (step time) per request; default to parameter
        dt = getattr(msg, 'dt', self.var_time_steps)
        try:
            dt = float(dt)
        except Exception:
            dt = self.var_time_steps

        steps = int(getattr(msg, 'prediction_steps', 0) or 0)
        if steps <= 0:
            # Nothing to do, publish empty (or a single repeat with low confidence)
            self.prediction_response_pub.publish(response)
            return

        if len(history) < self.min_history_length:
            # Fallback: repeat last position with conservative confidence
            if history:
                last_pos = history[-1]
                response.predicted_positions = [last_pos for _ in range(steps)]
                response.prediction_confidence = [0.2 for _ in range(steps)]
            else:
                # No history at all; emit origin points
                zero = Point()
                response.predicted_positions = [zero for _ in range(steps)]
                response.prediction_confidence = [0.0 for _ in range(steps)]
        else:
            preds = self.predict_positions(history, steps, dt)
            conf = self.compute_confidence(history, preds, dt)
            response.predicted_positions = preds
            response.prediction_confidence = conf

        self.prediction_response_pub.publish(response)
    
    def predict_positions(self, history: List[Point], steps: int, dt: float) -> List[Point]:
        """Predict future positions using a smoothed constant-velocity model.

        - Computes velocity over the last-K segments (velocity_window) and averages.
        - Uses provided dt per step.
        - Returns a list of Points of length `steps`.
        """
        if steps <= 0:
            return []

        if len(history) < 2:
            last_pos = history[-1] if history else Point()
            return [last_pos] * steps

        # Convert to numpy for easier computation
        positions = np.array([[p.x, p.y] for p in history], dtype=float)

        # Compute per-step velocity over last-K segments, then average
        k = min(self.velocity_window, len(positions) - 1)
        diffs = positions[-k:] - positions[-k-1:-1]
        # If dt is extremely small (bad input), clamp to parameter default
        step_dt = dt if dt and dt > 1e-6 else self.var_time_steps
        v = np.mean(diffs / step_dt, axis=0)

        # Predict future positions
        last_position = positions[-1]
        predictions: List[Point] = []
        for i in range(1, steps + 1):
            future_pos = last_position + v * (i * step_dt)
            p = Point()
            p.x = float(future_pos[0])
            p.y = float(future_pos[1])
            p.z = 0.0
            predictions.append(p)
        return predictions
    
    def predict_with_var_model(self, history, steps):
        """
        Placeholder for future VAR (Vector Autoregression) implementation.
        This method will implement more sophisticated prediction logic.
        """
        # TODO: Implement VAR model here
        # For now, fall back to constant velocity
        return self.predict_positions(history, steps, self.var_time_steps)

    def compute_confidence(self, history: List[Point], predictions: List[Point], dt: float) -> List[float]:
        """Compute simple confidence scores per-step.

        Heuristic: start from confidence_base and decay with step index and
        limited by short history or low apparent speed.
        """
        n_hist = len(history)
        base = self.confidence_base

        # Speed magnitude estimate
        if n_hist >= 2:
            dx = history[-1].x - history[-2].x
            dy = history[-1].y - history[-2].y
            step_dt = dt if dt and dt > 1e-6 else self.var_time_steps
            speed = np.hypot(dx, dy) / max(step_dt, 1e-6)
        else:
            speed = 0.0

        # History penalty: fewer points -> lower base
        if n_hist < 5:
            base *= 0.8
        if n_hist < 3:
            base *= 0.7

        # Low-speed penalty (harder to estimate heading)
        if speed < 0.05:
            base *= 0.8

        # Per-step exponential decay
        conf = []
        for i, _ in enumerate(predictions, start=1):
            c = max(0.05, min(0.98, base * (0.95 ** (i - 1))))
            conf.append(float(c))
        return conf

def main(args=None):
    rclpy.init(args=args)
    node = PredictorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
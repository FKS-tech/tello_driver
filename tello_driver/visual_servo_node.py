#!/usr/bin/env python3

import json
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


class VisualServoNode(Node):
    def __init__(self):
        super().__init__('visual_servo_node')

        self.declare_parameter('target_class_name', '')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('min_area_ratio', 0.0)
        self.declare_parameter('max_area_ratio', 1.0)
        self.declare_parameter('yaw_kp', 35.0)
        self.declare_parameter('max_yaw_speed', 30.0)
        self.declare_parameter('center_deadband', 0.10)
        self.declare_parameter('detection_timeout', 0.5)
        self.declare_parameter('publish_zero_when_lost', True)
        self.declare_parameter('target_selection_strategy', 'closest_to_center')
        self.declare_parameter('enable_target_lock', True)
        self.declare_parameter('target_lock_timeout', 1.0)
        self.declare_parameter('target_lock_max_error_distance', 0.35)
        self.declare_parameter('input_detection_topic', '/vision/detections')

        self.input_detection_topic = self.get_parameter('input_detection_topic').value
        self.target_class_name = self.get_parameter('target_class_name').value
        self.confidence_threshold = float(self.get_parameter('confidence_threshold').value)
        self.min_area_ratio = float(self.get_parameter('min_area_ratio').value)
        self.max_area_ratio = float(self.get_parameter('max_area_ratio').value)
        self.yaw_kp = float(self.get_parameter('yaw_kp').value)
        self.max_yaw_speed = float(self.get_parameter('max_yaw_speed').value)
        self.center_deadband = float(self.get_parameter('center_deadband').value)
        self.detection_timeout = float(self.get_parameter('detection_timeout').value)
        self.publish_zero_when_lost = bool(self.get_parameter('publish_zero_when_lost').value)
        self.target_selection_strategy = str(
            self.get_parameter('target_selection_strategy').value
        ).strip()
        self.enable_target_lock = bool(self.get_parameter('enable_target_lock').value)
        self.target_lock_timeout = float(self.get_parameter('target_lock_timeout').value)
        self.target_lock_max_error_distance = float(
            self.get_parameter('target_lock_max_error_distance').value
        )

        valid_strategies = {'highest_confidence', 'largest_area', 'closest_to_center'}
        if self.target_selection_strategy not in valid_strategies:
            self.get_logger().warn(
                'target_selection_strategy invalida: '
                f'{self.target_selection_strategy}. Usando closest_to_center.'
            )
            self.target_selection_strategy = 'closest_to_center'

        self.detection_sub = self.create_subscription(
            String,
            self.input_detection_topic,
            self._detections_callback,
            10,
        )
        self.cmd_pub = self.create_publisher(Twist, '/tello/autonomy/cmd_vel', 10)
        self.debug_pub = self.create_publisher(String, '/tello/autonomy/debug', 10)

        self.node_start_time_ns = self.get_clock().now().nanoseconds
        self.last_valid_detection_time_ns = 0
        self.target_lost = True
        self.last_invalid_warning_time_ns = 0
        self.last_target_error_x = None
        self.last_target_error_y = None
        self.last_target_lock_time_ns = 0

        self.watchdog_timer = self.create_timer(0.1, self._watchdog_callback)

        self.get_logger().info('Visual servo node iniciado em modo seguro')
        self.get_logger().info(f'Assinando deteccoes em: {self.input_detection_topic}')
        self.get_logger().info('Publicando comando visual em: /tello/autonomy/cmd_vel')

    @staticmethod
    def _clamp(value: float, min_value: float, max_value: float) -> float:
        return max(min_value, min(max_value, value))

    @staticmethod
    def _zero_twist() -> Twist:
        return Twist()

    def _warn_invalid_message(self, reason: str) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_invalid_warning_time_ns < int(2.0 * 1e9):
            return

        self.get_logger().warn(
            f'Mensagem invalida em {self.input_detection_topic}: {reason}'
        )
        self.last_invalid_warning_time_ns = now_ns

    def _publish_debug(self, payload: dict) -> None:
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.debug_pub.publish(msg)

    def _publish_no_target_debug(self, reason: str) -> None:
        self._publish_debug({
            'target_found': False,
            'reason': reason,
            'selection_strategy': self.target_selection_strategy,
            'target_lock_enabled': self.enable_target_lock,
            'yaw_cmd': 0.0,
        })

    def _detections_callback(self, msg: String) -> None:
        try:
            detections = json.loads(msg.data)
        except json.JSONDecodeError:
            self._warn_invalid_message('json_decode_error')
            return

        if not isinstance(detections, list):
            self._warn_invalid_message('payload_not_list')
            return

        target, target_lock_used = self._select_target(detections)
        if target is None:
            self.target_lost = True
            if self.publish_zero_when_lost:
                self.cmd_pub.publish(self._zero_twist())
            self._publish_no_target_debug('no_valid_detection')
            return

        error_x, error_y = self._get_detection_error(target)
        yaw_cmd = self._compute_yaw_command(error_x)

        twist = self._zero_twist()
        twist.angular.z = yaw_cmd
        self.cmd_pub.publish(twist)

        now_ns = self.get_clock().now().nanoseconds
        self.last_valid_detection_time_ns = now_ns
        self.last_target_error_x = error_x
        self.last_target_error_y = error_y
        self.last_target_lock_time_ns = now_ns
        self.target_lost = False

        self._publish_debug({
            'target_found': True,
            'selection_strategy': self.target_selection_strategy,
            'target_lock_enabled': self.enable_target_lock,
            'target_lock_used': target_lock_used,
            'class_name': target.get('class_name', ''),
            'confidence': float(target['confidence']),
            'area_ratio': float(target['area_ratio']),
            'error_x': error_x,
            'error_y': error_y,
            'yaw_cmd': yaw_cmd,
        })

    def _select_target(self, detections: list) -> tuple[Optional[dict], bool]:
        valid_detections = []

        for detection in detections:
            if not isinstance(detection, dict):
                continue

            if not self._is_valid_detection(detection):
                continue

            valid_detections.append(detection)

        if not valid_detections:
            return None, False

        now_ns = self.get_clock().now().nanoseconds
        if self.enable_target_lock and self._has_recent_target_lock(now_ns):
            locked_target = min(valid_detections, key=self._target_error_distance)
            if self._target_error_distance(locked_target) <= self.target_lock_max_error_distance:
                return locked_target, True

        if self.target_selection_strategy == 'highest_confidence':
            return max(
                valid_detections,
                key=lambda detection: (
                    float(detection['confidence']),
                    float(detection['area_ratio']),
                ),
            ), False

        if self.target_selection_strategy == 'largest_area':
            return max(
                valid_detections,
                key=lambda detection: (
                    float(detection['area_ratio']),
                    float(detection['confidence']),
                ),
            ), False

        return min(
            valid_detections,
            key=lambda detection: (
                self._center_distance(detection),
                -float(detection['confidence']),
            ),
        ), False

    def _is_valid_detection(self, detection: dict) -> bool:
        try:
            confidence = float(detection.get('confidence'))
            area_ratio = float(detection.get('area_ratio'))
            self._get_detection_error(detection)
        except (TypeError, ValueError):
            return False

        if confidence < self.confidence_threshold:
            return False

        if area_ratio < self.min_area_ratio or area_ratio > self.max_area_ratio:
            return False

        class_name = str(detection.get('class_name', ''))
        if self.target_class_name and class_name != self.target_class_name:
            return False

        return True

    def _get_detection_error(self, detection: dict) -> tuple[float, float]:
        error_norm = detection.get('error_norm')
        if not isinstance(error_norm, list) or len(error_norm) < 1:
            raise ValueError('invalid error_norm')

        error_x = float(error_norm[0])
        error_y = float(error_norm[1]) if len(error_norm) > 1 else 0.0
        return error_x, error_y

    def _has_recent_target_lock(self, now_ns: int) -> bool:
        if self.last_target_error_x is None or self.last_target_error_y is None:
            return False

        if self.last_target_lock_time_ns == 0:
            return False

        age_sec = (now_ns - self.last_target_lock_time_ns) / 1e9
        return age_sec <= self.target_lock_timeout

    def _target_error_distance(self, detection: dict) -> float:
        error_x, error_y = self._get_detection_error(detection)
        dx = error_x - self.last_target_error_x
        dy = error_y - self.last_target_error_y
        return (dx * dx + dy * dy) ** 0.5

    def _center_distance(self, detection: dict) -> float:
        error_x, error_y = self._get_detection_error(detection)
        return (error_x * error_x + error_y * error_y) ** 0.5

    def _compute_yaw_command(self, error_x: float) -> float:
        if abs(error_x) < self.center_deadband:
            return 0.0

        yaw_cmd = self.yaw_kp * error_x
        return self._clamp(yaw_cmd, -self.max_yaw_speed, self.max_yaw_speed)

    def _watchdog_callback(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        reference_time_ns = self.last_valid_detection_time_ns or self.node_start_time_ns
        age_sec = (now_ns - reference_time_ns) / 1e9

        if age_sec <= self.detection_timeout:
            return

        if self.publish_zero_when_lost:
            self.cmd_pub.publish(self._zero_twist())

        reason = 'detection_timeout' if self.last_valid_detection_time_ns else 'no_valid_detection'
        self._publish_no_target_debug(reason)

        if not self.target_lost:
            self.get_logger().warn('Alvo visual perdido por timeout.')
            self.target_lost = True


def main(args=None):
    rclpy.init(args=args)
    node = VisualServoNode()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

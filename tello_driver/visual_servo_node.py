#!/usr/bin/env python3

import json
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
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

        self.target_class_name = self.get_parameter('target_class_name').value
        self.confidence_threshold = float(self.get_parameter('confidence_threshold').value)
        self.min_area_ratio = float(self.get_parameter('min_area_ratio').value)
        self.max_area_ratio = float(self.get_parameter('max_area_ratio').value)
        self.yaw_kp = float(self.get_parameter('yaw_kp').value)
        self.max_yaw_speed = float(self.get_parameter('max_yaw_speed').value)
        self.center_deadband = float(self.get_parameter('center_deadband').value)
        self.detection_timeout = float(self.get_parameter('detection_timeout').value)
        self.publish_zero_when_lost = bool(self.get_parameter('publish_zero_when_lost').value)

        self.detection_sub = self.create_subscription(
            String,
            '/vision/detections',
            self._detections_callback,
            10,
        )
        self.cmd_pub = self.create_publisher(Twist, '/tello/autonomy/cmd_vel', 10)
        self.debug_pub = self.create_publisher(String, '/tello/autonomy/debug', 10)

        self.node_start_time_ns = self.get_clock().now().nanoseconds
        self.last_valid_detection_time_ns = 0
        self.target_lost = True
        self.last_invalid_warning_time_ns = 0

        self.watchdog_timer = self.create_timer(0.1, self._watchdog_callback)

        self.get_logger().info('Visual servo node iniciado em modo seguro')
        self.get_logger().info('Assinando deteccoes em: /vision/detections')
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

        self.get_logger().warn(f'Mensagem invalida em /vision/detections: {reason}')
        self.last_invalid_warning_time_ns = now_ns

    def _publish_debug(self, payload: dict) -> None:
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.debug_pub.publish(msg)

    def _publish_no_target_debug(self, reason: str) -> None:
        self._publish_debug({
            'target_found': False,
            'reason': reason,
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

        target = self._select_target(detections)
        if target is None:
            self.target_lost = True
            if self.publish_zero_when_lost:
                self.cmd_pub.publish(self._zero_twist())
            self._publish_no_target_debug('no_valid_detection')
            return

        error_x = float(target['error_norm'][0])
        yaw_cmd = self._compute_yaw_command(error_x)

        twist = self._zero_twist()
        twist.angular.z = yaw_cmd
        self.cmd_pub.publish(twist)

        self.last_valid_detection_time_ns = self.get_clock().now().nanoseconds
        self.target_lost = False

        self._publish_debug({
            'target_found': True,
            'class_name': target.get('class_name', ''),
            'confidence': float(target['confidence']),
            'area_ratio': float(target['area_ratio']),
            'error_x': error_x,
            'yaw_cmd': yaw_cmd,
        })

    def _select_target(self, detections: list) -> Optional[dict]:
        valid_detections = []

        for detection in detections:
            if not isinstance(detection, dict):
                continue

            if not self._is_valid_detection(detection):
                continue

            valid_detections.append(detection)

        if not valid_detections:
            return None

        return max(
            valid_detections,
            key=lambda detection: (
                float(detection['confidence']),
                float(detection['area_ratio']),
            ),
        )

    def _is_valid_detection(self, detection: dict) -> bool:
        try:
            confidence = float(detection.get('confidence'))
            area_ratio = float(detection.get('area_ratio'))
            error_norm = detection.get('error_norm')
        except (TypeError, ValueError):
            return False

        if confidence < self.confidence_threshold:
            return False

        if area_ratio < self.min_area_ratio or area_ratio > self.max_area_ratio:
            return False

        class_name = str(detection.get('class_name', ''))
        if self.target_class_name and class_name != self.target_class_name:
            return False

        if not isinstance(error_norm, list) or len(error_norm) < 1:
            return False

        try:
            float(error_norm[0])
        except (TypeError, ValueError):
            return False

        return True

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
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

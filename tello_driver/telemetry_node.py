#!/usr/bin/env python3

import json
import socket

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TelloTelemetryNode(Node):
    def __init__(self):
        super().__init__('tello_telemetry_node')

        self.declare_parameter('telemetry_port', 8890)
        self.declare_parameter('raw_topic', '/tello/telemetry/raw')
        self.declare_parameter('json_topic', '/tello/telemetry/json')
        self.declare_parameter('poll_period', 0.05)
        self.declare_parameter('log_summary', True)

        self.telemetry_port = int(self.get_parameter('telemetry_port').value)
        self.raw_topic = self.get_parameter('raw_topic').value
        self.json_topic = self.get_parameter('json_topic').value
        self.poll_period = float(self.get_parameter('poll_period').value)
        self.log_summary = bool(self.get_parameter('log_summary').value)

        self.raw_pub = self.create_publisher(String, self.raw_topic, 10)
        self.json_pub = self.create_publisher(String, self.json_topic, 10)

        self.telemetry_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.telemetry_socket.bind(('', self.telemetry_port))
        self.telemetry_socket.setblocking(False)

        self.timer = self.create_timer(self.poll_period, self._poll_telemetry)

        self.last_summary_log_ns = 0
        self.summary_log_interval_ns = int(2.0 * 1e9)

        self.get_logger().info('Tello telemetry node iniciado')
        self.get_logger().info(f'Escutando telemetria UDP na porta {self.telemetry_port}')
        self.get_logger().info(f'Publicando bruto em: {self.raw_topic}')
        self.get_logger().info(f'Publicando JSON em: {self.json_topic}')

    def _parse_telemetry(self, data: str) -> dict:
        parsed = {}

        for item in data.strip().split(';'):
            if ':' not in item:
                continue

            key, value = item.split(':', 1)
            parsed[key] = self._convert_value(value)

        return parsed

    @staticmethod
    def _convert_value(value: str):
        try:
            if '.' in value:
                return float(value)
            return int(value)
        except ValueError:
            return value

    def _poll_telemetry(self):
        while True:
            try:
                data, _ = self.telemetry_socket.recvfrom(2048)
            except BlockingIOError:
                break
            except Exception as exc:
                self.get_logger().error(f'Erro ao ler telemetria: {exc}')
                break

            decoded = data.decode('utf-8', errors='ignore').strip()
            if not decoded:
                continue

            parsed = self._parse_telemetry(decoded)

            raw_msg = String()
            raw_msg.data = decoded
            self.raw_pub.publish(raw_msg)

            json_msg = String()
            json_msg.data = json.dumps(parsed, ensure_ascii=False)
            self.json_pub.publish(json_msg)

            if self.log_summary:
                self._log_summary(parsed)

    def _log_summary(self, telemetry: dict):
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_summary_log_ns < self.summary_log_interval_ns:
            return

        bat = telemetry.get('bat', 'N/A')
        h = telemetry.get('h', 'N/A')
        yaw = telemetry.get('yaw', 'N/A')
        pitch = telemetry.get('pitch', 'N/A')
        roll = telemetry.get('roll', 'N/A')
        tof = telemetry.get('tof', 'N/A')

        self.get_logger().info(
            f'Telemetria | bat={bat}% | h={h}cm | tof={tof}cm | '
            f'yaw={yaw} | pitch={pitch} | roll={roll}'
        )

        self.last_summary_log_ns = now_ns

    def destroy_node(self):
        try:
            self.telemetry_socket.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TelloTelemetryNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
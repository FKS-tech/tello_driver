#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import socket


class TelemetryViewer(Node):

    def __init__(self):
        super().__init__('telemetry_viewer')

        # socket da telemetria
        self.telemetry_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.telemetry_socket.bind(('', 8890))
        self.telemetry_socket.setblocking(False)

        self.telemetry_publisher = self.create_publisher(String, 'tello/telemetry', 10)
        self.telemetry_timer = self.create_timer(0.2, self.publish_telemetry)

        self.subscription = self.create_subscription(String, 'tello/telemetry', self.callback, 10)
        

        self.get_logger().info("Telemetry Viewer iniciado")

    def read_telemetry(self):
        try:
            data, _ = self.telemetry_socket.recvfrom(1024)
            decoded = data.decode('utf-8')
            return decoded
        except BlockingIOError:
            return None
        
    def publish_telemetry(self):
        data = self.read_telemetry()

        if data:
            msg = String()
            msg.data = data
            self.telemetry_publisher.publish(msg)

    def parse(self, data):
        parsed = {}

        for item in data.split(';'):
            if ':' in item:
                key, value = item.split(':')
                parsed[key] = value

        return parsed

    def callback(self, msg):
        data = self.parse(msg.data)

        try:
            bat = data.get("bat", "N/A")
            h = data.get("h", "N/A")
            yaw = data.get("yaw", "N/A")
            pitch = data.get("pitch", "N/A")
            roll = data.get("roll", "N/A")

            print("\n===== TELEMETRIA =====")
            print(f"Bateria: {bat}%")
            print(f"Altura: {h} cm")
            print(f"Yaw: {yaw}")
            print(f"Pitch: {pitch}")
            print(f"Roll: {roll}")

        except Exception as e:
            self.get_logger().error(f"Erro ao processar: {e}")


def main(args=None):
    rclpy.init(args=args)

    node = TelemetryViewer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
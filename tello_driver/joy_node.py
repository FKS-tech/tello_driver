#!/usr/bin/env python3

from typing import Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

from tello_driver.tello_client import TelloClient


class TelloJoyNode(Node):
    def __init__(self):
        super().__init__('tello_joy_node')

        # -----------------------------
        # Parâmetros do Tello
        # -----------------------------
        self.declare_parameter('tello_ip', '192.168.10.1')
        self.declare_parameter('tello_port', 8889)
        self.declare_parameter('local_port', 9000)
        self.declare_parameter('sdk_timeout', 5.0)

        # -----------------------------
        # Parâmetros do joystick
        # -----------------------------
        self.declare_parameter('joy_topic', '/joy')

        self.declare_parameter('axis_yaw', 0)
        self.declare_parameter('axis_up_down', 1)
        self.declare_parameter('axis_left_right', 3)
        self.declare_parameter('axis_forward_back', 4)

        self.declare_parameter('button_land', 0)
        self.declare_parameter('button_takeoff', 3)

        # -----------------------------
        # Ajustes de controle
        # -----------------------------
        self.declare_parameter('deadzone', 0.30)
        self.declare_parameter('rc_period', 0.05)
        self.declare_parameter('joy_timeout', 0.30)

        self.tello_ip = self.get_parameter('tello_ip').value
        self.tello_port = int(self.get_parameter('tello_port').value)
        self.local_port = int(self.get_parameter('local_port').value)
        self.sdk_timeout = float(self.get_parameter('sdk_timeout').value)

        self.joy_topic = self.get_parameter('joy_topic').value

        self.axis_yaw = int(self.get_parameter('axis_yaw').value)
        self.axis_up_down = int(self.get_parameter('axis_up_down').value)
        self.axis_left_right = int(self.get_parameter('axis_left_right').value)
        self.axis_forward_back = int(self.get_parameter('axis_forward_back').value)

        self.button_land = int(self.get_parameter('button_land').value)
        self.button_takeoff = int(self.get_parameter('button_takeoff').value)

        self.deadzone = float(self.get_parameter('deadzone').value)
        self.rc_period = float(self.get_parameter('rc_period').value)
        self.joy_timeout = float(self.get_parameter('joy_timeout').value)

        # Cliente do Tello
        self.tello = TelloClient(
            tello_ip=self.tello_ip,
            tello_port=self.tello_port,
            local_port=self.local_port,
            timeout=self.sdk_timeout,
        )

        # Estado
        self.current_rc: Tuple[int, int, int, int] = (0, 0, 0, 0)
        self.last_joy_time_ns = 0
        self.last_takeoff_pressed = False
        self.last_land_pressed = False
        self.sdk_ready = False

        # Inicialização
        self._initialize_tello()

        # ROS interfaces
        self.joy_sub = self.create_subscription(
            Joy,
            self.joy_topic,
            self._joy_callback,
            10
        )

        self.rc_timer = self.create_timer(self.rc_period, self._send_current_rc)

        self.get_logger().info('Tello joy node iniciado')
        self.get_logger().info(f'Assinando joystick em: {self.joy_topic}')

    def _initialize_tello(self) -> None:
        response = self.tello.enter_sdk_mode()

        if response is None:
            self.sdk_ready = False
            self.get_logger().error('Falha ao entrar em modo SDK.')
            return

        self.sdk_ready = True
        self.get_logger().info(f'SDK ativo: {response}')

    def _apply_deadzone(self, value: float) -> float:
        return 0.0 if abs(value) < self.deadzone else value

    @staticmethod
    def _safe_axis(axes, index: int) -> float:
        if index < 0 or index >= len(axes):
            return 0.0
        return axes[index]

    @staticmethod
    def _safe_button(buttons, index: int) -> int:
        if index < 0 or index >= len(buttons):
            return 0
        return buttons[index]

    def _joy_callback(self, msg: Joy) -> None:
        self.last_joy_time_ns = self.get_clock().now().nanoseconds

        yaw = self._apply_deadzone(self._safe_axis(msg.axes, self.axis_yaw)) * -100.0
        up_down = self._apply_deadzone(self._safe_axis(msg.axes, self.axis_up_down)) * 100.0
        left_right = self._apply_deadzone(self._safe_axis(msg.axes, self.axis_left_right)) * -100.0
        forward_back = self._apply_deadzone(self._safe_axis(msg.axes, self.axis_forward_back)) * 100.0

        self.current_rc = (
            int(left_right),
            int(forward_back),
            int(up_down),
            int(yaw),
        )

        takeoff_pressed = bool(self._safe_button(msg.buttons, self.button_takeoff))
        land_pressed = bool(self._safe_button(msg.buttons, self.button_land))

        if takeoff_pressed and not self.last_takeoff_pressed:
            response = self.tello.takeoff()
            self.get_logger().warn(f'Takeoff: {response}')

        if land_pressed and not self.last_land_pressed:
            response = self.tello.land()
            self.get_logger().warn(f'Land: {response}')

        self.last_takeoff_pressed = takeoff_pressed
        self.last_land_pressed = land_pressed

    def _send_current_rc(self) -> None:
        if not self.sdk_ready:
            return

        now_ns = self.get_clock().now().nanoseconds

        if self.last_joy_time_ns == 0:
            self.tello.send_rc(0, 0, 0, 0)
            return

        age_sec = (now_ns - self.last_joy_time_ns) / 1e9
        if age_sec > self.joy_timeout:
            self.current_rc = (0, 0, 0, 0)

        ok = self.tello.send_rc(*self.current_rc)
        if not ok:
            self.get_logger().warn('Falha ao enviar comando RC.')

    def destroy_node(self):
        try:
            self.tello.send_rc(0, 0, 0, 0)
        except Exception:
            pass

        self.tello.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TelloJoyNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
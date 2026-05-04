#!/usr/bin/env python3

from typing import Optional, Tuple

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
        self.declare_parameter('command_cooldown', 3.0)
        self.declare_parameter('takeoff_rc_pause', 3.0)
        self.declare_parameter('land_rc_pause', 2.0)

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
        self.command_cooldown = float(self.get_parameter('command_cooldown').value)
        self.takeoff_rc_pause = float(self.get_parameter('takeoff_rc_pause').value)
        self.land_rc_pause = float(self.get_parameter('land_rc_pause').value)

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
        self.is_flying: Optional[bool] = None
        self.last_critical_command_time_ns = 0
        self.rc_pause_until_ns = 0

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

    @staticmethod
    def _is_ok_response(response: Optional[str]) -> bool:
        return response is not None and response.strip().lower() == 'ok'

    def _cooldown_remaining(self, now_ns: int) -> float:
        if self.last_critical_command_time_ns == 0:
            return 0.0

        elapsed_sec = (now_ns - self.last_critical_command_time_ns) / 1e9
        return max(0.0, self.command_cooldown - elapsed_sec)

    def _pause_rc(self, duration_sec: float) -> None:
        now_ns = self.get_clock().now().nanoseconds
        pause_until_ns = now_ns + int(duration_sec * 1e9)
        self.rc_pause_until_ns = max(self.rc_pause_until_ns, pause_until_ns)
        self.current_rc = (0, 0, 0, 0)

    def _handle_takeoff(self, now_ns: int) -> None:
        remaining = self._cooldown_remaining(now_ns)
        if remaining > 0.0:
            self.get_logger().warn(
                f'Takeoff ignorado: cooldown ativo ({remaining:.1f}s restantes).'
            )
            return

        if self.is_flying is True:
            self.get_logger().warn('Takeoff ignorado: drone ja esta marcado como em voo.')
            return

        self.last_critical_command_time_ns = now_ns
        response = self.tello.takeoff()
        self.get_logger().warn(f'Takeoff: {response}')

        if self._is_ok_response(response):
            self.is_flying = True
            self._pause_rc(self.takeoff_rc_pause)

    def _handle_land(self, now_ns: int) -> None:
        remaining = self._cooldown_remaining(now_ns)
        if remaining > 0.0:
            self.get_logger().warn(
                f'Land ignorado: cooldown ativo ({remaining:.1f}s restantes).'
            )
            return

        if self.is_flying is False:
            self.get_logger().warn('Land ignorado: drone ja esta marcado como no solo.')
            return

        self.last_critical_command_time_ns = now_ns
        response = self.tello.land()
        self.get_logger().warn(f'Land: {response}')

        if self._is_ok_response(response):
            self.is_flying = False
            self._pause_rc(self.land_rc_pause)

    def _joy_callback(self, msg: Joy) -> None:
        now_ns = self.get_clock().now().nanoseconds
        self.last_joy_time_ns = now_ns

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
            self._handle_takeoff(now_ns)

        if land_pressed and not self.last_land_pressed:
            self._handle_land(now_ns)

        self.last_takeoff_pressed = takeoff_pressed
        self.last_land_pressed = land_pressed

    def _send_current_rc(self) -> None:
        if not self.sdk_ready:
            return

        now_ns = self.get_clock().now().nanoseconds

        if now_ns < self.rc_pause_until_ns:
            self.current_rc = (0, 0, 0, 0)
            self.tello.send_rc(0, 0, 0, 0)
            return

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

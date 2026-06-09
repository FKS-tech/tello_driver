#!/usr/bin/env python3

import math
from typing import Callable, Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Empty

from tello_driver.tello_client import TelloClient


ZERO_RC: Tuple[int, int, int, int] = (0, 0, 0, 0)


class CommandMuxNode(Node):
    def __init__(self):
        super().__init__('command_mux_node')

        self.declare_parameter('tello_ip', '192.168.10.1')
        self.declare_parameter('tello_port', 8889)
        self.declare_parameter('local_port', 9002)
        self.declare_parameter('sdk_timeout', 5.0)
        self.declare_parameter('enable_sdk_init', True)

        self.declare_parameter('cmd_vel_topic', '/tello/autonomy/cmd_vel')
        self.declare_parameter('takeoff_topic', '/tello/autonomy/takeoff')
        self.declare_parameter('land_topic', '/tello/autonomy/land')

        self.declare_parameter('rc_period', 0.05)
        self.declare_parameter('watchdog_timeout', 0.4)
        self.declare_parameter('command_cooldown', 3.0)
        self.declare_parameter('takeoff_rc_pause', 3.0)
        self.declare_parameter('land_rc_pause', 2.0)

        self.tello_ip = self.get_parameter('tello_ip').value
        self.tello_port = int(self.get_parameter('tello_port').value)
        self.local_port = int(self.get_parameter('local_port').value)
        self.sdk_timeout = float(self.get_parameter('sdk_timeout').value)
        self.enable_sdk_init = bool(self.get_parameter('enable_sdk_init').value)

        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.takeoff_topic = self.get_parameter('takeoff_topic').value
        self.land_topic = self.get_parameter('land_topic').value

        self.rc_period = float(self.get_parameter('rc_period').value)
        self.watchdog_timeout = float(self.get_parameter('watchdog_timeout').value)
        self.command_cooldown = float(self.get_parameter('command_cooldown').value)
        self.takeoff_rc_pause = float(self.get_parameter('takeoff_rc_pause').value)
        self.land_rc_pause = float(self.get_parameter('land_rc_pause').value)

        self.tello = TelloClient(
            tello_ip=self.tello_ip,
            tello_port=self.tello_port,
            local_port=self.local_port,
            timeout=self.sdk_timeout,
        )

        self.current_rc = ZERO_RC
        self.last_cmd_time_ns = 0
        self.last_critical_command_time_ns = 0
        self.last_rc_failure_warn_time_ns = 0
        self.rc_pause_until_ns = 0
        self.watchdog_stopped = False
        self.sdk_ready = False
        self.is_flying: Optional[bool] = None

        self._initialize_tello()

        self.cmd_sub = self.create_subscription(
            Twist,
            self.cmd_vel_topic,
            self._cmd_vel_callback,
            10,
        )
        self.takeoff_sub = self.create_subscription(
            Empty,
            self.takeoff_topic,
            self._takeoff_callback,
            10,
        )
        self.land_sub = self.create_subscription(
            Empty,
            self.land_topic,
            self._land_callback,
            10,
        )

        self.rc_timer = self.create_timer(self.rc_period, self._rc_timer_callback)

        self.get_logger().info('Command mux node iniciado')
        self.get_logger().info(f'Assinando comandos em: {self.cmd_vel_topic}')
        self.get_logger().info(f'Takeoff por topico: {self.takeoff_topic}')
        self.get_logger().info(f'Land por topico: {self.land_topic}')
        self.get_logger().info(
            f'Watchdog RC: {self.watchdog_timeout:.2f}s, periodo: {self.rc_period:.2f}s'
        )

    def _initialize_tello(self) -> None:
        if not self.enable_sdk_init:
            self.sdk_ready = True
            self.get_logger().info(
                'SDK init desativado; enviando comandos assumindo SDK ja ativo.'
            )
            return

        response = self.tello.enter_sdk_mode()
        if self._is_ok_response(response):
            self.sdk_ready = True
            self.get_logger().info(f'SDK ativo: {response}')
            return

        self.sdk_ready = False
        self.get_logger().error(f'Falha ao entrar em modo SDK: {response}')

    @staticmethod
    def _is_ok_response(response: Optional[str]) -> bool:
        return response is not None and response.strip().lower() == 'ok'

    @staticmethod
    def _to_rc_axis(value: float) -> int:
        if not math.isfinite(value):
            return 0

        return max(-100, min(100, int(value)))

    def _twist_to_rc(self, msg: Twist) -> Tuple[int, int, int, int]:
        forward_back = self._to_rc_axis(msg.linear.x)
        left_right = self._to_rc_axis(msg.linear.y)
        up_down = self._to_rc_axis(msg.linear.z)
        yaw = self._to_rc_axis(msg.angular.z)

        return left_right, forward_back, up_down, yaw

    def _cmd_vel_callback(self, msg: Twist) -> None:
        self.current_rc = self._twist_to_rc(msg)
        self.last_cmd_time_ns = self.get_clock().now().nanoseconds
        self.watchdog_stopped = False

        if not self.sdk_ready:
            return

        if self._is_rc_paused(self.last_cmd_time_ns):
            return

        self._send_rc(*self.current_rc)

    def _takeoff_callback(self, _msg: Empty) -> None:
        self._handle_critical_command(
            name='Takeoff',
            command=self.tello.takeoff,
            pause_sec=self.takeoff_rc_pause,
            flying_state_on_success=True,
        )

    def _land_callback(self, _msg: Empty) -> None:
        self._handle_critical_command(
            name='Land',
            command=self.tello.land,
            pause_sec=self.land_rc_pause,
            flying_state_on_success=False,
        )

    def _cooldown_remaining(self, now_ns: int) -> float:
        if self.last_critical_command_time_ns == 0:
            return 0.0

        elapsed_sec = (now_ns - self.last_critical_command_time_ns) / 1e9
        return max(0.0, self.command_cooldown - elapsed_sec)

    def _pause_rc(self, duration_sec: float) -> None:
        now_ns = self.get_clock().now().nanoseconds
        self.rc_pause_until_ns = max(
            self.rc_pause_until_ns,
            now_ns + int(duration_sec * 1e9),
        )
        self.current_rc = ZERO_RC
        self._send_rc(*ZERO_RC)

    def _handle_critical_command(
        self,
        name: str,
        command: Callable[[], Optional[str]],
        pause_sec: float,
        flying_state_on_success: bool,
    ) -> None:
        if not self.sdk_ready:
            self.get_logger().error(f'{name} ignorado: SDK nao esta pronto.')
            return

        now_ns = self.get_clock().now().nanoseconds
        remaining = self._cooldown_remaining(now_ns)
        if remaining > 0.0:
            self.get_logger().warn(
                f'{name} ignorado: cooldown ativo ({remaining:.1f}s restantes).'
            )
            return

        if name == 'Takeoff' and self.is_flying is True:
            self.get_logger().warn('Takeoff ignorado: drone ja esta marcado como em voo.')
            return

        if name == 'Land' and self.is_flying is False:
            self.get_logger().warn('Land ignorado: drone ja esta marcado como no solo.')
            return

        self.last_critical_command_time_ns = now_ns
        self._pause_rc(pause_sec)

        response = command()
        self.get_logger().warn(f'{name}: {response}')

        if self._is_ok_response(response):
            self.is_flying = flying_state_on_success

        self._pause_rc(pause_sec)

    def _is_rc_paused(self, now_ns: int) -> bool:
        return now_ns < self.rc_pause_until_ns

    def _rc_timer_callback(self) -> None:
        if not self.sdk_ready:
            return

        now_ns = self.get_clock().now().nanoseconds

        if self._is_rc_paused(now_ns):
            self.current_rc = ZERO_RC
            self._send_rc(*ZERO_RC)
            return

        if self.last_cmd_time_ns == 0:
            self._send_rc(*ZERO_RC)
            return

        age_sec = (now_ns - self.last_cmd_time_ns) / 1e9
        if age_sec > self.watchdog_timeout:
            if not self.watchdog_stopped and self.current_rc != ZERO_RC:
                self.get_logger().warn(
                    f'Watchdog RC: sem cmd_vel novo por {age_sec:.2f}s; enviando zero.'
                )

            self.watchdog_stopped = True
            self.current_rc = ZERO_RC
            self._send_rc(*ZERO_RC)
            return

        self._send_rc(*self.current_rc)

    def _warn_rc_failure(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_rc_failure_warn_time_ns < int(1.0 * 1e9):
            return

        self.get_logger().warn('Falha ao enviar comando RC ao Tello.')
        self.last_rc_failure_warn_time_ns = now_ns

    def _send_rc(
        self,
        left_right: int,
        forward_back: int,
        up_down: int,
        yaw: int,
    ) -> bool:
        ok = self.tello.send_rc(left_right, forward_back, up_down, yaw)
        if not ok:
            self._warn_rc_failure()

        return ok

    def destroy_node(self):
        try:
            self.tello.send_rc(*ZERO_RC)
        except Exception:
            pass

        self.tello.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CommandMuxNode()

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

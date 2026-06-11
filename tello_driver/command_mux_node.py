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
    """Ponte segura entre comandos ROS autonomos e comandos reais do Tello."""

    def __init__(self):
        """Declara parametros, inicializa SDK e cria topicos de controle."""
        super().__init__('command_mux_node')

        self.declare_parameter('tello_ip', '192.168.10.1')
        self.declare_parameter('tello_port', 8889)
        self.declare_parameter('local_port', 9002)
        self.declare_parameter('sdk_timeout', 5.0)
        self.declare_parameter('enable_sdk_init', True)

        self.declare_parameter('cmd_vel_topic', '/tello/autonomy/cmd_vel')
        self.declare_parameter('takeoff_topic', '/tello/autonomy/takeoff')
        self.declare_parameter('land_topic', '/tello/autonomy/land')
        self.declare_parameter('emergency_topic', '/tello/autonomy/emergency')
        self.declare_parameter('enable_topic', '/tello/autonomy/enable')
        self.declare_parameter('disable_topic', '/tello/autonomy/disable')
        self.declare_parameter('stop_topic', '/tello/autonomy/stop')

        self.declare_parameter('rc_period', 0.05)
        self.declare_parameter('watchdog_timeout', 0.4)
        self.declare_parameter('command_cooldown', 3.0)
        self.declare_parameter('takeoff_rc_pause', 3.0)
        self.declare_parameter('land_rc_pause', 2.0)
        self.declare_parameter('start_armed', False)
        self.declare_parameter('max_xy_speed', 30.0)
        self.declare_parameter('max_z_speed', 25.0)
        self.declare_parameter('max_yaw_speed', 30.0)

        self.tello_ip = self.get_parameter('tello_ip').value
        self.tello_port = int(self.get_parameter('tello_port').value)
        self.local_port = int(self.get_parameter('local_port').value)
        self.sdk_timeout = float(self.get_parameter('sdk_timeout').value)
        self.enable_sdk_init = bool(self.get_parameter('enable_sdk_init').value)

        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.takeoff_topic = self.get_parameter('takeoff_topic').value
        self.land_topic = self.get_parameter('land_topic').value
        self.emergency_topic = self.get_parameter('emergency_topic').value
        self.enable_topic = self.get_parameter('enable_topic').value
        self.disable_topic = self.get_parameter('disable_topic').value
        self.stop_topic = self.get_parameter('stop_topic').value

        self.rc_period = float(self.get_parameter('rc_period').value)
        self.watchdog_timeout = float(self.get_parameter('watchdog_timeout').value)
        self.command_cooldown = float(self.get_parameter('command_cooldown').value)
        self.takeoff_rc_pause = float(self.get_parameter('takeoff_rc_pause').value)
        self.land_rc_pause = float(self.get_parameter('land_rc_pause').value)
        self.armed = bool(self.get_parameter('start_armed').value)
        self.max_xy_speed = self._safe_speed_limit(
            self.get_parameter('max_xy_speed').value
        )
        self.max_z_speed = self._safe_speed_limit(
            self.get_parameter('max_z_speed').value
        )
        self.max_yaw_speed = self._safe_speed_limit(
            self.get_parameter('max_yaw_speed').value
        )

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
        self.last_disarmed_cmd_warn_time_ns = 0
        self.last_limit_warn_time_ns = 0
        self.rc_pause_until_ns = 0
        self.watchdog_stopped = False
        self.sdk_ready = False
        self.is_flying: Optional[bool] = None

        self._initialize_tello()
        self._send_zero_rc()

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
        self.emergency_sub = self.create_subscription(
            Empty,
            self.emergency_topic,
            self._emergency_callback,
            10,
        )
        self.enable_sub = self.create_subscription(
            Empty,
            self.enable_topic,
            self._enable_callback,
            10,
        )
        self.disable_sub = self.create_subscription(
            Empty,
            self.disable_topic,
            self._disable_callback,
            10,
        )
        self.stop_sub = self.create_subscription(
            Empty,
            self.stop_topic,
            self._stop_callback,
            10,
        )

        self.rc_timer = self.create_timer(self.rc_period, self._rc_timer_callback)

        self.get_logger().info('Command mux node iniciado')
        self.get_logger().info(f'Assinando comandos em: {self.cmd_vel_topic}')
        self.get_logger().info(f'Takeoff por topico: {self.takeoff_topic}')
        self.get_logger().info(f'Land por topico: {self.land_topic}')
        self.get_logger().info(f'Emergency por topico: {self.emergency_topic}')
        self.get_logger().info(
            f'Estado inicial: {"armado" if self.armed else "desarmado"}'
        )
        self.get_logger().info(
            f'Watchdog RC: {self.watchdog_timeout:.2f}s, periodo: {self.rc_period:.2f}s'
        )
        self.get_logger().info(
            'Limites RC: '
            f'xy={self.max_xy_speed:.0f}, z={self.max_z_speed:.0f}, '
            f'yaw={self.max_yaw_speed:.0f}'
        )

    def _initialize_tello(self) -> None:
        """Inicializa o SDK ou assume que outro no ja fez essa etapa."""
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
        """Verifica se uma resposta textual do SDK equivale a `ok`."""
        return response is not None and response.strip().lower() == 'ok'

    @staticmethod
    def _safe_speed_limit(value: float) -> float:
        """Normaliza limites de velocidade para a faixa segura 0..100."""
        try:
            speed_limit = float(value)
        except (TypeError, ValueError):
            return 0.0

        if not math.isfinite(speed_limit):
            return 0.0

        return max(0.0, min(100.0, abs(speed_limit)))

    @staticmethod
    def _clamp(value: float, min_value: float, max_value: float) -> float:
        """Limita um valor dentro de um intervalo fechado."""
        return max(min_value, min(max_value, value))

    def _warn_disarmed_cmd(self) -> None:
        """Avisa com throttle quando cmd_vel chega com autonomia desarmada."""
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_disarmed_cmd_warn_time_ns < int(2.0 * 1e9):
            return

        self.get_logger().warn('cmd_vel ignorado: autonomia esta desarmada.')
        self.last_disarmed_cmd_warn_time_ns = now_ns

    def _warn_speed_limited(
        self,
        axis_name: str,
        requested_value: float,
        limited_value: float,
    ) -> None:
        """Avisa com throttle quando algum eixo de RC foi limitado."""
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_limit_warn_time_ns < int(2.0 * 1e9):
            return

        self.get_logger().warn(
            f'Limite aplicado em {axis_name}: '
            f'{requested_value:.1f} -> {limited_value:.1f}'
        )
        self.last_limit_warn_time_ns = now_ns

    def _to_limited_rc_axis(
        self,
        value: float,
        speed_limit: float,
        axis_name: str,
    ) -> int:
        """Converte um eixo Twist em inteiro RC respeitando limites."""
        if not math.isfinite(value):
            return 0

        absolute_limited = self._clamp(value, -100.0, 100.0)
        axis_limited = self._clamp(absolute_limited, -speed_limit, speed_limit)

        if axis_limited != value:
            self._warn_speed_limited(axis_name, value, axis_limited)

        return int(axis_limited)

    def _twist_to_rc(self, msg: Twist) -> Tuple[int, int, int, int]:
        """Mapeia Twist para a ordem RC do Tello: lr, fb, ud, yaw."""
        forward_back = self._to_limited_rc_axis(
            msg.linear.x,
            self.max_xy_speed,
            'linear.x',
        )
        left_right = self._to_limited_rc_axis(
            msg.linear.y,
            self.max_xy_speed,
            'linear.y',
        )
        up_down = self._to_limited_rc_axis(
            msg.linear.z,
            self.max_z_speed,
            'linear.z',
        )
        yaw = self._to_limited_rc_axis(
            msg.angular.z,
            self.max_yaw_speed,
            'angular.z',
        )

        return left_right, forward_back, up_down, yaw

    def _cmd_vel_callback(self, msg: Twist) -> None:
        """Recebe cmd_vel autonomo e envia RC se a autonomia estiver armada."""
        if not self.armed:
            self.current_rc = ZERO_RC
            self._send_zero_rc()
            self._warn_disarmed_cmd()
            return

        self.current_rc = self._twist_to_rc(msg)
        self.last_cmd_time_ns = self.get_clock().now().nanoseconds
        self.watchdog_stopped = False

        if not self.sdk_ready:
            return

        if self._is_rc_paused(self.last_cmd_time_ns):
            return

        self._send_rc(*self.current_rc)

    def _takeoff_callback(self, _msg: Empty) -> None:
        """Executa takeoff por topico, exigindo autonomia armada."""
        self._handle_critical_command(
            name='Takeoff',
            command=self.tello.takeoff,
            pause_sec=self.takeoff_rc_pause,
            flying_state_on_success=True,
            requires_armed=True,
            uses_cooldown=True,
        )

    def _land_callback(self, _msg: Empty) -> None:
        """Executa land por topico, mesmo com autonomia desarmada."""
        self._handle_critical_command(
            name='Land',
            command=self.tello.land,
            pause_sec=self.land_rc_pause,
            flying_state_on_success=False,
            requires_armed=False,
            uses_cooldown=False,
        )

    def _enable_callback(self, _msg: Empty) -> None:
        """Arma a autonomia para aceitar cmd_vel."""
        if self.armed:
            self.get_logger().info('Enable recebido: autonomia ja estava armada.')
            return

        self.armed = True
        self.current_rc = ZERO_RC
        self.last_cmd_time_ns = 0
        self.watchdog_stopped = False
        self._send_zero_rc()
        self.get_logger().warn('Autonomia armada: cmd_vel sera aceito.')

    def _disable_callback(self, _msg: Empty) -> None:
        """Desarma a autonomia e zera o RC imediatamente."""
        was_armed = self.armed
        self.armed = False
        self.current_rc = ZERO_RC
        self.last_cmd_time_ns = 0
        self.watchdog_stopped = False
        self._send_zero_rc()

        if was_armed:
            self.get_logger().warn('Autonomia desarmada: RC zerado.')
        else:
            self.get_logger().info('Disable recebido: autonomia ja estava desarmada.')

    def _stop_callback(self, _msg: Empty) -> None:
        """Zera o RC sem alterar o estado armado/desarmado."""
        self.current_rc = ZERO_RC
        self.watchdog_stopped = False
        self._send_zero_rc()
        self.get_logger().warn(
            f'Stop recebido: RC zerado; autonomia continua '
            f'{"armada" if self.armed else "desarmada"}.'
        )

    def _emergency_callback(self, _msg: Empty) -> None:
        """Envia emergency ao Tello e desarma a autonomia."""
        self.current_rc = ZERO_RC
        self.armed = False
        self.watchdog_stopped = False
        self._send_zero_rc()

        response = self.tello.emergency()
        if self._is_ok_response(response):
            self.is_flying = False
            self.get_logger().error(
                'Emergency enviado ao Tello: resposta ok. Autonomia desarmada.'
            )
            return

        self.get_logger().error(
            f'Emergency enviado ao Tello, mas resposta nao foi ok: {response}. '
            'Autonomia desarmada; confirme o estado fisico do drone.'
        )

    def _cooldown_remaining(self, now_ns: int) -> float:
        """Calcula cooldown restante para comandos criticos."""
        if self.last_critical_command_time_ns == 0:
            return 0.0

        elapsed_sec = (now_ns - self.last_critical_command_time_ns) / 1e9
        return max(0.0, self.command_cooldown - elapsed_sec)

    def _pause_rc(self, duration_sec: float) -> None:
        """Pausa comandos RC apos takeoff/land para evitar interferencia."""
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
        requires_armed: bool,
        uses_cooldown: bool,
    ) -> None:
        """Centraliza validacoes e execucao de takeoff/land/emergency-like."""
        if requires_armed and not self.armed:
            self.get_logger().warn(f'{name} ignorado: autonomia esta desarmada.')
            return

        if not self.sdk_ready:
            self.get_logger().error(f'{name} ignorado: SDK nao esta pronto.')
            return

        now_ns = self.get_clock().now().nanoseconds
        if uses_cooldown:
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
        """Indica se RC deve permanecer zerado por pausa temporaria."""
        return now_ns < self.rc_pause_until_ns

    def _rc_timer_callback(self) -> None:
        """Reenvia RC atual ou zero conforme watchdog, pausa e estado armado."""
        now_ns = self.get_clock().now().nanoseconds

        if not self.armed:
            self.current_rc = ZERO_RC
            self._send_zero_rc()
            return

        if not self.sdk_ready:
            return

        if self._is_rc_paused(now_ns):
            self.current_rc = ZERO_RC
            self._send_zero_rc()
            return

        if self.last_cmd_time_ns == 0:
            self._send_zero_rc()
            return

        age_sec = (now_ns - self.last_cmd_time_ns) / 1e9
        if age_sec > self.watchdog_timeout:
            if not self.watchdog_stopped and self.current_rc != ZERO_RC:
                self.get_logger().warn(
                    f'Watchdog RC: sem cmd_vel novo por {age_sec:.2f}s; enviando zero.'
                )

            self.watchdog_stopped = True
            self.current_rc = ZERO_RC
            self._send_zero_rc()
            return

        self._send_rc(*self.current_rc)

    def _warn_rc_failure(self) -> None:
        """Avisa com throttle quando o envio RC falha."""
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
        """Envia um comando RC ao Tello e registra falhas."""
        ok = self.tello.send_rc(left_right, forward_back, up_down, yaw)
        if not ok:
            self._warn_rc_failure()

        return ok

    def _send_zero_rc(self) -> bool:
        """Atalho para enviar RC totalmente zerado."""
        return self._send_rc(*ZERO_RC)

    def destroy_node(self):
        """Zera RC e fecha o cliente Tello durante o shutdown."""
        try:
            self.tello.send_rc(*ZERO_RC)
        except Exception:
            pass

        self.tello.close()
        super().destroy_node()


def main(args=None):
    """Start command_mux_node and keep it spinning until shutdown."""
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

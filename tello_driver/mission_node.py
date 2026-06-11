#!/usr/bin/env python3

import json
import math
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, Empty, String

from tello_driver.visual_math import clamp


PHASE1_DEMO = 'phase1_demo'
SUPPORTED_MISSIONS = {PHASE1_DEMO}


class MissionNode(Node):
    """Maquina de estados segura para demonstrar a missao de base em dry_run."""

    def __init__(self):
        """Configura parametros, topicos de controle/status e timer principal."""
        super().__init__('mission_node')

        self.declare_parameter('mission_id', PHASE1_DEMO)
        self.declare_parameter('dry_run', True)
        self.declare_parameter('auto_start', False)
        self.declare_parameter('target_class_name', 'landing_base')

        self.declare_parameter('telemetry_topic', '/tello/telemetry/json')
        self.declare_parameter('landing_base_topic', '/vision/landing_base')

        self.declare_parameter('cmd_vel_topic', '/tello/autonomy/cmd_vel')
        self.declare_parameter('enable_topic', '/tello/autonomy/enable')
        self.declare_parameter('disable_topic', '/tello/autonomy/disable')
        self.declare_parameter('stop_topic', '/tello/autonomy/stop')
        self.declare_parameter('takeoff_topic', '/tello/autonomy/takeoff')
        self.declare_parameter('land_topic', '/tello/autonomy/land')

        self.declare_parameter('status_topic', '/mission/status')
        self.declare_parameter('event_topic', '/mission/event')
        self.declare_parameter('map_topic', '/mission/map')
        self.declare_parameter('start_topic', '/mission/start')
        self.declare_parameter('abort_topic', '/mission/abort')
        self.declare_parameter('reset_topic', '/mission/reset')

        self.declare_parameter('control_rate_hz', 10.0)
        self.declare_parameter('min_battery_percent', 35)

        self.declare_parameter('stabilize_duration_sec', 3.0)
        self.declare_parameter('scan_duration_sec', 10.0)
        self.declare_parameter('scan_yaw_speed', 15.0)

        self.declare_parameter('align_error_x_threshold', 0.15)
        self.declare_parameter('align_yaw_gain', 25.0)
        self.declare_parameter('max_yaw_cmd', 20.0)

        self.declare_parameter('approach_speed', 10.0)
        self.declare_parameter('landing_area_ratio_threshold', 0.28)
        self.declare_parameter('detection_timeout_sec', 1.0)

        self.mission_id = self.get_parameter('mission_id').value
        self.dry_run = bool(self.get_parameter('dry_run').value)
        self.auto_start = bool(self.get_parameter('auto_start').value)
        self.target_class_name = self.get_parameter('target_class_name').value

        self.telemetry_topic = self.get_parameter('telemetry_topic').value
        self.landing_base_topic = self.get_parameter('landing_base_topic').value

        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.enable_topic = self.get_parameter('enable_topic').value
        self.disable_topic = self.get_parameter('disable_topic').value
        self.stop_topic = self.get_parameter('stop_topic').value
        self.takeoff_topic = self.get_parameter('takeoff_topic').value
        self.land_topic = self.get_parameter('land_topic').value

        self.status_topic = self.get_parameter('status_topic').value
        self.event_topic = self.get_parameter('event_topic').value
        self.map_topic = self.get_parameter('map_topic').value
        self.start_topic = self.get_parameter('start_topic').value
        self.abort_topic = self.get_parameter('abort_topic').value
        self.reset_topic = self.get_parameter('reset_topic').value

        self.control_rate_hz = max(1.0, float(
            self.get_parameter('control_rate_hz').value,
        ))
        self.min_battery_percent = int(
            self.get_parameter('min_battery_percent').value,
        )
        self.stabilize_duration_sec = float(
            self.get_parameter('stabilize_duration_sec').value,
        )
        self.scan_duration_sec = float(
            self.get_parameter('scan_duration_sec').value,
        )
        self.scan_yaw_speed = float(self.get_parameter('scan_yaw_speed').value)
        self.align_error_x_threshold = float(
            self.get_parameter('align_error_x_threshold').value,
        )
        self.align_yaw_gain = float(self.get_parameter('align_yaw_gain').value)
        self.max_yaw_cmd = abs(float(self.get_parameter('max_yaw_cmd').value))
        self.approach_speed = float(self.get_parameter('approach_speed').value)
        self.landing_area_ratio_threshold = float(
            self.get_parameter('landing_area_ratio_threshold').value,
        )
        self.detection_timeout_sec = float(
            self.get_parameter('detection_timeout_sec').value,
        )

        self.state = 'IDLE'
        self.state_entered_ns = self.get_clock().now().nanoseconds
        self.started = self.auto_start
        self.abort_requested = False
        self.selected_base_id: Optional[str] = None
        self.base_map = []
        self.next_base_id = 1
        self.last_telemetry = {}
        self.last_detections = []
        self.last_detection_time_ns = 0
        self.last_target_detection = None
        self.last_cmd_preview = self._empty_cmd_preview()
        self.last_abort_reason = None
        self.abort_commands_sent = False

        self.telemetry_sub = self.create_subscription(
            String,
            self.telemetry_topic,
            self._telemetry_callback,
            10,
        )
        self.landing_base_sub = self.create_subscription(
            String,
            self.landing_base_topic,
            self._landing_base_callback,
            10,
        )
        self.start_sub = self.create_subscription(
            Bool,
            self.start_topic,
            self._start_callback,
            10,
        )
        self.abort_sub = self.create_subscription(
            Empty,
            self.abort_topic,
            self._abort_callback,
            10,
        )
        self.reset_sub = self.create_subscription(
            Empty,
            self.reset_topic,
            self._reset_callback,
            10,
        )

        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.enable_pub = self.create_publisher(Empty, self.enable_topic, 10)
        self.disable_pub = self.create_publisher(Empty, self.disable_topic, 10)
        self.stop_pub = self.create_publisher(Empty, self.stop_topic, 10)
        self.takeoff_pub = self.create_publisher(Empty, self.takeoff_topic, 10)
        self.land_pub = self.create_publisher(Empty, self.land_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.event_pub = self.create_publisher(String, self.event_topic, 10)
        self.map_pub = self.create_publisher(String, self.map_topic, 10)

        self.timer = self.create_timer(
            1.0 / self.control_rate_hz,
            self._timer_callback,
        )

        self.get_logger().info('Mission node iniciado')
        self.get_logger().info(f'Missao: {self.mission_id}')
        self.get_logger().info(f'Dry run: {self.dry_run}')
        self.get_logger().info(f'Deteccao de base: {self.landing_base_topic}')

    def _telemetry_callback(self, msg: String) -> None:
        """Atualiza a ultima telemetria JSON recebida do Tello."""
        try:
            telemetry = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warn(f'Telemetria JSON invalida: {exc}')
            return

        if isinstance(telemetry, dict):
            self.last_telemetry = telemetry

    def _landing_base_callback(self, msg: String) -> None:
        """Atualiza a lista de deteccoes de base vindas do detector visual."""
        try:
            detections = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warn(f'Deteccao de base JSON invalida: {exc}')
            return

        if not isinstance(detections, list):
            return

        self.last_detections = [
            detection
            for detection in detections
            if isinstance(detection, dict)
        ]
        if self.last_detections:
            self.last_detection_time_ns = self.get_clock().now().nanoseconds

    def _start_callback(self, msg: Bool) -> None:
        """Recebe pedido externo para iniciar a missao."""
        if not msg.data:
            return

        self.started = True
        self._publish_event('start_requested')

    def _abort_callback(self, _msg: Empty) -> None:
        """Recebe pedido externo de abortar a missao com seguranca."""
        self.abort_requested = True
        self.last_abort_reason = 'abort_topic'
        self._publish_event('abort_requested')

    def _reset_callback(self, _msg: Empty) -> None:
        """Volta a missao para IDLE e limpa mapa/estado interno."""
        self._reset_mission()
        self._publish_event('mission_reset')

    def _timer_callback(self) -> None:
        """Executa a maquina de estados e publica status/mapa a cada ciclo."""
        now_ns = self.get_clock().now().nanoseconds

        if self.abort_requested and self.state != 'ABORT':
            self._transition('ABORT', reason=self.last_abort_reason or 'abort')

        if self.state not in ('IDLE', 'FINISHED', 'ABORT'):
            low_battery = self._battery_is_low()
            if low_battery:
                self.last_abort_reason = 'low_battery'
                self._transition('ABORT', reason='low_battery')

        if self.state == 'IDLE':
            self._handle_idle()
        elif self.state == 'ARM':
            self._handle_arm()
        elif self.state == 'TAKEOFF':
            self._handle_takeoff()
        elif self.state == 'STABILIZE':
            self._handle_stabilize(now_ns)
        elif self.state == 'SCAN_ARENA':
            self._handle_scan_arena(now_ns)
        elif self.state == 'SELECT_BASE':
            self._handle_select_base()
        elif self.state == 'ALIGN_BASE':
            self._handle_align_base()
        elif self.state == 'APPROACH_BASE':
            self._handle_approach_base()
        elif self.state == 'LAND':
            self._handle_land()
        elif self.state == 'ABORT':
            self._handle_abort()

        self._publish_status()
        self._publish_map()

    def _handle_idle(self) -> None:
        """Espera start e valida se a missao solicitada existe."""
        self.last_cmd_preview = self._empty_cmd_preview()
        if not self.started:
            return

        if self.mission_id not in SUPPORTED_MISSIONS:
            self._publish_event(
                'mission_not_implemented',
                requested_mission_id=self.mission_id,
            )
            self.started = False
            return

        self._transition('ARM')

    def _handle_arm(self) -> None:
        """Arma autonomia ou apenas registra o que faria em dry_run."""
        if not self.dry_run:
            self.enable_pub.publish(Empty())
        self._publish_event('would_enable_autonomy' if self.dry_run else 'enable_autonomy')
        self._transition('TAKEOFF')

    def _handle_takeoff(self) -> None:
        """Solicita takeoff ou apenas registra o que faria em dry_run."""
        if not self.dry_run:
            self.takeoff_pub.publish(Empty())
        self._publish_event('would_takeoff' if self.dry_run else 'takeoff')
        self._transition('STABILIZE')

    def _handle_stabilize(self, now_ns: int) -> None:
        """Espera alguns segundos apos takeoff antes de procurar base."""
        self.last_cmd_preview = self._empty_cmd_preview()
        if self._state_age_sec(now_ns) >= self.stabilize_duration_sec:
            self._transition('SCAN_ARENA')

    def _handle_scan_arena(self, now_ns: int) -> None:
        """Gira devagar procurando bases e registrando observacoes no mapa."""
        self._remember_current_base_observations()

        cmd = self._build_twist(yaw=self.scan_yaw_speed)
        self._publish_or_preview_cmd(cmd)

        if self._state_age_sec(now_ns) >= self.scan_duration_sec:
            self._transition('SELECT_BASE')

    def _handle_select_base(self) -> None:
        """Escolhe a melhor base vista ou aborta se nenhuma foi encontrada."""
        selected = self._select_base_from_map()
        if selected is None:
            self.last_abort_reason = 'no_base_detected'
            self._publish_event('no_base_detected')
            self._transition('ABORT', reason='no_base_detected')
            return

        self.selected_base_id = selected['id']
        self._publish_event('base_selected', base=selected)
        self._transition('ALIGN_BASE')

    def _handle_align_base(self) -> None:
        """Alinha yaw usando error_norm.x da base selecionada/visivel."""
        target = self._best_current_detection()
        if target is None or self._detection_is_stale():
            self._publish_event('base_lost_returning_to_scan')
            self._transition('SCAN_ARENA')
            return

        self._remember_base_observation(target)
        error_x = self._safe_float(target.get('error_norm', [0.0, 0.0])[0])
        yaw_cmd = self._compute_yaw_cmd(error_x)
        cmd = self._build_twist(yaw=yaw_cmd)
        self._publish_or_preview_cmd(cmd)

        if abs(error_x) <= self.align_error_x_threshold:
            self._transition('APPROACH_BASE')

    def _handle_approach_base(self) -> None:
        """Avanca devagar enquanto base esta alinhada e visivel."""
        target = self._best_current_detection()
        if target is None or self._detection_is_stale():
            self._publish_event('base_lost_returning_to_align')
            self._transition('ALIGN_BASE')
            return

        self._remember_base_observation(target)
        error_x = self._safe_float(target.get('error_norm', [0.0, 0.0])[0])
        area_ratio = self._safe_float(target.get('area_ratio', 0.0))

        if abs(error_x) > self.align_error_x_threshold:
            self._transition('ALIGN_BASE')
            return

        yaw_cmd = self._compute_yaw_cmd(error_x)
        cmd = self._build_twist(linear_x=self.approach_speed, yaw=yaw_cmd)
        self._publish_or_preview_cmd(cmd)

        if area_ratio >= self.landing_area_ratio_threshold:
            self._transition('LAND')

    def _handle_land(self) -> None:
        """Solicita pouso ou apenas registra o pouso em dry_run."""
        self._publish_or_preview_cmd(self._build_twist())
        if not self.dry_run:
            self.land_pub.publish(Empty())
        self._publish_event('would_land' if self.dry_run else 'land')
        self._transition('FINISHED')

    def _handle_abort(self) -> None:
        """Para/desarma uma vez quando a missao entra em ABORT."""
        self.last_cmd_preview = self._empty_cmd_preview()
        if self.abort_commands_sent:
            return

        if not self.dry_run:
            self.stop_pub.publish(Empty())
            self.disable_pub.publish(Empty())
        self.abort_commands_sent = True

    def _transition(self, new_state: str, **data) -> None:
        """Troca de estado, reinicia cronometro do estado e publica evento."""
        if self.state == new_state:
            return

        previous_state = self.state
        self.state = new_state
        self.state_entered_ns = self.get_clock().now().nanoseconds
        if new_state == 'ABORT':
            self.abort_commands_sent = False
        self._publish_event(
            'state_transition',
            previous_state=previous_state,
            state=new_state,
            **data,
        )

    def _reset_mission(self) -> None:
        """Limpa estado de execucao e prepara a missao para novo start."""
        self.state = 'IDLE'
        self.state_entered_ns = self.get_clock().now().nanoseconds
        self.started = self.auto_start
        self.abort_requested = False
        self.selected_base_id = None
        self.base_map = []
        self.next_base_id = 1
        self.last_detections = []
        self.last_detection_time_ns = 0
        self.last_target_detection = None
        self.last_cmd_preview = self._empty_cmd_preview()
        self.last_abort_reason = None
        self.abort_commands_sent = False

    def _best_current_detection(self) -> Optional[dict]:
        """Seleciona a melhor deteccao atual da classe alvo configurada."""
        candidates = [
            detection
            for detection in self.last_detections
            if detection.get('class_name') == self.target_class_name
        ]
        if not candidates:
            return None

        return max(candidates, key=self._detection_score)

    @staticmethod
    def _detection_score(detection: dict) -> float:
        """Calcula score simples para ordenar deteccoes atuais."""
        confidence = MissionNode._safe_float(detection.get('confidence', 0.0))
        area_ratio = MissionNode._safe_float(detection.get('area_ratio', 0.0))
        pattern_score = MissionNode._safe_float(detection.get('pattern_score', 0.0))
        return confidence + area_ratio + 0.25 * pattern_score

    def _remember_base_observation(self, detection: dict) -> None:
        """Atualiza o mapa visual simples com uma observacao de base."""
        now_ns = self.get_clock().now().nanoseconds
        self.last_target_detection = detection

        center = self._safe_point(detection.get('center_px'))
        if center is None:
            return

        frame_size = detection.get('frame_size') or [1, 1]
        frame_w = max(1.0, self._safe_float(frame_size[0]))
        frame_h = max(1.0, self._safe_float(frame_size[1]))
        center_norm = [center[0] / frame_w, center[1] / frame_h]

        existing = self._find_nearby_map_entry(
            detection.get('class_name', 'unknown'),
            center_norm,
        )
        if existing is None:
            existing = {
                'id': f'base_{self.next_base_id}',
                'class_name': detection.get('class_name', 'unknown'),
                'first_seen_sec': self._now_sec(),
                'observations': 0,
                'best_confidence': 0.0,
                'best_area_ratio': 0.0,
                'last_center_norm': center_norm,
                'last_error_norm': detection.get('error_norm'),
                'last_seen_sec': self._now_sec(),
            }
            self.next_base_id += 1
            self.base_map.append(existing)

        existing['observations'] += 1
        existing['last_seen_sec'] = self._now_sec()
        existing['last_seen_age_sec'] = 0.0
        existing['last_center_norm'] = center_norm
        existing['last_error_norm'] = detection.get('error_norm')
        existing['best_confidence'] = max(
            existing['best_confidence'],
            self._safe_float(detection.get('confidence', 0.0)),
        )
        existing['best_area_ratio'] = max(
            existing['best_area_ratio'],
            self._safe_float(detection.get('area_ratio', 0.0)),
        )
        existing['last_observation_time_ns'] = now_ns

    def _remember_current_base_observations(self) -> None:
        """Registra todas as bases visuais presentes no frame atual."""
        for detection in self.last_detections:
            class_name = detection.get('class_name')
            if class_name not in ('landing_base', 'takeoff_base'):
                continue
            self._remember_base_observation(detection)

    def _find_nearby_map_entry(
        self,
        class_name: str,
        center_norm: list[float],
    ) -> Optional[dict]:
        """Procura no mapa uma base da mesma classe perto da posicao visual."""
        best_entry = None
        best_distance = float('inf')

        for entry in self.base_map:
            if entry.get('class_name') != class_name:
                continue

            previous = entry.get('last_center_norm')
            if not previous:
                continue

            dx = center_norm[0] - previous[0]
            dy = center_norm[1] - previous[1]
            distance = math.sqrt(dx * dx + dy * dy)
            if distance < best_distance:
                best_distance = distance
                best_entry = entry

        if best_distance <= 0.25:
            return best_entry
        return None

    def _select_base_from_map(self) -> Optional[dict]:
        """Escolhe do mapa a melhor base da classe alvo para aproximacao."""
        targets = [
            entry
            for entry in self.base_map
            if entry.get('class_name') == self.target_class_name
        ]
        if not targets:
            return None

        return max(
            targets,
            key=lambda entry: (
                entry.get('best_confidence', 0.0),
                entry.get('best_area_ratio', 0.0),
                entry.get('observations', 0),
            ),
        )

    def _publish_or_preview_cmd(self, cmd: Twist) -> None:
        """Atualiza cmd_preview e publica cmd_vel somente fora de dry_run."""
        self.last_cmd_preview = {
            'linear_x': cmd.linear.x,
            'linear_y': cmd.linear.y,
            'linear_z': cmd.linear.z,
            'angular_z': cmd.angular.z,
        }
        if not self.dry_run:
            self.cmd_vel_pub.publish(cmd)

    def _publish_status(self) -> None:
        """Publica um snapshot JSON do estado atual da missao."""
        now_ns = self.get_clock().now().nanoseconds
        target = self._best_current_detection()
        status = {
            'mission_id': self.mission_id,
            'state': self.state,
            'dry_run': self.dry_run,
            'target_class_name': self.target_class_name,
            'battery': self.last_telemetry.get('bat'),
            'height_cm': self.last_telemetry.get('h'),
            'tof_cm': self.last_telemetry.get('tof'),
            'base_visible': target is not None and not self._detection_is_stale(),
            'selected_base_id': self.selected_base_id,
            'map_size': len(self.base_map),
            'state_age_sec': self._state_age_sec(now_ns),
            'last_detection_age_sec': self._last_detection_age_sec(now_ns),
            'cmd_preview': self.last_cmd_preview,
            'abort_reason': self.last_abort_reason,
        }

        if target is not None:
            status.update({
                'error_x': self._safe_float(
                    target.get('error_norm', [0.0, 0.0])[0],
                ),
                'error_y': self._safe_float(
                    target.get('error_norm', [0.0, 0.0])[1],
                ),
                'area_ratio': self._safe_float(target.get('area_ratio', 0.0)),
                'confidence': self._safe_float(target.get('confidence', 0.0)),
            })

        msg = String()
        msg.data = json.dumps(status, ensure_ascii=False)
        self.status_pub.publish(msg)

    def _publish_event(self, event: str, **data) -> None:
        """Publica eventos discretos como transicoes, start e abort."""
        payload = {
            'mission_id': self.mission_id,
            'state': self.state,
            'dry_run': self.dry_run,
            'event': event,
            **data,
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.event_pub.publish(msg)

    def _publish_map(self) -> None:
        """Publica o mapa visual simples com idade de cada base vista."""
        now_ns = self.get_clock().now().nanoseconds
        bases = []
        for entry in self.base_map:
            last_time_ns = entry.get('last_observation_time_ns', now_ns)
            serialized = {
                key: value
                for key, value in entry.items()
                if key != 'last_observation_time_ns'
            }
            serialized['last_seen_age_sec'] = max(
                0.0,
                (now_ns - last_time_ns) / 1e9,
            )
            bases.append(serialized)

        msg = String()
        msg.data = json.dumps({
            'mission_id': self.mission_id,
            'state': self.state,
            'selected_base_id': self.selected_base_id,
            'bases': bases,
        }, ensure_ascii=False)
        self.map_pub.publish(msg)

    def _battery_is_low(self) -> bool:
        """Verifica se a telemetria indica bateria abaixo do minimo."""
        battery = self.last_telemetry.get('bat')
        if battery is None:
            return False

        try:
            return int(battery) < self.min_battery_percent
        except (TypeError, ValueError):
            return False

    def _detection_is_stale(self) -> bool:
        """Indica se a ultima deteccao visual ja passou do timeout."""
        age = self._last_detection_age_sec(self.get_clock().now().nanoseconds)
        if age is None:
            return True
        return age > self.detection_timeout_sec

    def _last_detection_age_sec(self, now_ns: int) -> Optional[float]:
        """Retorna idade da ultima deteccao ou None se nunca houve deteccao."""
        if self.last_detection_time_ns <= 0:
            return None
        return max(0.0, (now_ns - self.last_detection_time_ns) / 1e9)

    def _state_age_sec(self, now_ns: int) -> float:
        """Retorna ha quantos segundos a missao esta no estado atual."""
        return max(0.0, (now_ns - self.state_entered_ns) / 1e9)

    def _compute_yaw_cmd(self, error_x: float) -> float:
        """Converte erro horizontal em comando de yaw limitado."""
        if not math.isfinite(error_x):
            return 0.0

        yaw_cmd = self.align_yaw_gain * error_x
        return float(clamp(yaw_cmd, -self.max_yaw_cmd, self.max_yaw_cmd))

    @staticmethod
    def _build_twist(
        linear_x: float = 0.0,
        linear_y: float = 0.0,
        linear_z: float = 0.0,
        yaw: float = 0.0,
    ) -> Twist:
        """Cria Twist na escala RC usada pelo command_mux_node."""
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.linear.y = float(linear_y)
        msg.linear.z = float(linear_z)
        msg.angular.z = float(yaw)
        return msg

    @staticmethod
    def _empty_cmd_preview() -> dict:
        """Retorna o formato padrao de preview sem movimento."""
        return {
            'linear_x': 0.0,
            'linear_y': 0.0,
            'linear_z': 0.0,
            'angular_z': 0.0,
        }

    @staticmethod
    def _safe_float(value, default: float = 0.0) -> float:
        """Converte valor para float finito com fallback."""
        try:
            result = float(value)
        except (TypeError, ValueError):
            return default

        if not math.isfinite(result):
            return default
        return result

    @staticmethod
    def _safe_point(value) -> Optional[list[float]]:
        """Converte uma lista [x, y] em ponto float, ou None se invalida."""
        if not isinstance(value, list) or len(value) < 2:
            return None
        return [
            MissionNode._safe_float(value[0]),
            MissionNode._safe_float(value[1]),
        ]

    def _now_sec(self) -> float:
        """Retorna tempo atual do relogio ROS em segundos."""
        return self.get_clock().now().nanoseconds / 1e9


def main(args=None):
    """Start mission_node and run the selected mission state machine."""
    rclpy.init(args=args)
    node = MissionNode()

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

#!/usr/bin/env python3

import json
from typing import Optional

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from tello_driver.visual_math import compute_bbox_center, compute_normalized_error


class QRNode(Node):
    """Detecta QR Codes em frames do Tello e publica deteccoes JSON."""

    def __init__(self):
        """Configura parametros, detector OpenCV, publishers e subscriber."""
        super().__init__('qr_node')

        self.declare_parameter('input_topic', '/tello/image_raw')
        self.declare_parameter('output_detection_topic', '/vision/qr_codes')
        self.declare_parameter('output_image_topic', '/vision/qr_image_annotated')
        self.declare_parameter('debug_topic', '/vision/qr_debug')
        self.declare_parameter('show_preview', False)
        self.declare_parameter('process_every_n_frames', 1)
        self.declare_parameter('publish_empty', True)
        self.declare_parameter('publish_undecoded', False)
        self.declare_parameter('min_area_ratio', 0.0001)
        self.declare_parameter('use_preprocessing', True)
        self.declare_parameter('try_grayscale', True)
        self.declare_parameter('try_clahe', True)
        self.declare_parameter('try_adaptive_threshold', True)
        self.declare_parameter('try_upscale', True)
        self.declare_parameter('upscale_factor', 1.5)

        self.input_topic = self.get_parameter('input_topic').value
        self.output_detection_topic = self.get_parameter('output_detection_topic').value
        self.output_image_topic = self.get_parameter('output_image_topic').value
        self.debug_topic = self.get_parameter('debug_topic').value
        self.show_preview = bool(self.get_parameter('show_preview').value)
        self.process_every_n_frames = max(
            1,
            int(self.get_parameter('process_every_n_frames').value),
        )
        self.publish_empty = bool(self.get_parameter('publish_empty').value)
        self.publish_undecoded = bool(self.get_parameter('publish_undecoded').value)
        self.min_area_ratio = float(self.get_parameter('min_area_ratio').value)
        self.use_preprocessing = bool(self.get_parameter('use_preprocessing').value)
        self.try_grayscale = bool(self.get_parameter('try_grayscale').value)
        self.try_clahe = bool(self.get_parameter('try_clahe').value)
        self.try_adaptive_threshold = bool(
            self.get_parameter('try_adaptive_threshold').value,
        )
        self.try_upscale = bool(self.get_parameter('try_upscale').value)
        self.upscale_factor = max(1.0, float(self.get_parameter('upscale_factor').value))

        self.bridge = CvBridge()
        self.detector = cv2.QRCodeDetector()
        self.clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        self.frame_count = 0
        self.last_detector_warning_time_ns = 0
        self.last_decoded_value = None
        self.last_qr_seen_time_ns = 0

        self.image_sub = self.create_subscription(
            Image,
            self.input_topic,
            self._image_callback,
            10,
        )
        self.detection_pub = self.create_publisher(
            String,
            self.output_detection_topic,
            10,
        )
        self.image_pub = self.create_publisher(
            Image,
            self.output_image_topic,
            10,
        )
        self.debug_pub = self.create_publisher(String, self.debug_topic, 10)

        self.get_logger().info('QR node iniciado')
        self.get_logger().info(f'Assinando imagem em: {self.input_topic}')
        self.get_logger().info(f'Publicando QR Codes em: {self.output_detection_topic}')

    def _image_callback(self, msg: Image) -> None:
        """Recebe frames, detecta QR Codes e publica debug/imagem anotada."""
        self.frame_count += 1
        if self.frame_count % self.process_every_n_frames != 0:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().error(f'Erro ao converter imagem: {exc}')
            return

        if frame is None or frame.size == 0:
            self._warn_detector('frame vazio recebido')
            return

        detections, annotated, detection_method = self._detect_and_annotate(frame)

        if detections or self.publish_empty:
            self._publish_detections(detections)

        self._publish_debug(detections, detection_method)
        self._publish_annotated_image(annotated, msg.header)

        if self.show_preview:
            cv2.imshow('QR Preview', annotated)
            cv2.waitKey(1)

    def _detect_and_annotate(self, frame):
        """Tenta detectar QR em varias versoes do frame e desenha resultados."""
        annotated = frame.copy()
        frame_h, frame_w = frame.shape[:2]
        detections = []
        detection_method = None

        for method, detection_frame in self._build_detection_frames(frame):
            method_detections = []
            method_points = []

            for data, points in self._detect_qr_points(detection_frame):
                original_points = self._scale_points_to_original(
                    points,
                    detection_frame,
                    frame_w,
                    frame_h,
                )
                detection = self._build_detection(
                    data,
                    original_points,
                    frame_w,
                    frame_h,
                )
                if detection is None:
                    continue

                method_detections.append(detection)
                method_points.append((original_points, detection))

            if not method_detections:
                continue

            detections = method_detections
            detection_method = method
            for original_points, detection in method_points:
                self._draw_detection(annotated, original_points, detection)
            break

        self._remember_decoded_qr(detections)

        return detections, annotated, detection_method

    def _build_detection_frames(self, frame) -> list[tuple[str, object]]:
        """Monta a sequencia de imagens tentadas pelo detector de QR."""
        frames = [('original', frame)]
        if not self.use_preprocessing:
            return frames

        gray = self._to_gray(frame)
        if gray is None:
            return frames

        if self.try_grayscale:
            frames.append(('gray', gray))

        if self.try_clahe:
            try:
                frames.append(('clahe', self.clahe.apply(gray)))
            except cv2.error as exc:
                self._warn_detector(f'CLAHE falhou: {exc}')

        if self.try_adaptive_threshold:
            try:
                threshold = cv2.adaptiveThreshold(
                    gray,
                    255,
                    cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                    cv2.THRESH_BINARY,
                    31,
                    2,
                )
                frames.append(('adaptive_threshold', threshold))
            except cv2.error as exc:
                self._warn_detector(f'adaptiveThreshold falhou: {exc}')

        if self.try_upscale and self.upscale_factor > 1.0:
            try:
                upscaled = cv2.resize(
                    frame,
                    None,
                    fx=self.upscale_factor,
                    fy=self.upscale_factor,
                    interpolation=cv2.INTER_CUBIC,
                )
                frames.append(('upscaled', upscaled))
            except cv2.error as exc:
                self._warn_detector(f'upscale falhou: {exc}')

        return frames

    @staticmethod
    def _to_gray(frame):
        """Converte frame BGR para cinza quando o preprocessamento precisa."""
        try:
            if len(frame.shape) == 2:
                return frame
            return cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        except (AttributeError, cv2.error):
            return None

    @staticmethod
    def _scale_points_to_original(points, detection_frame, frame_w: int, frame_h: int):
        """Reescala pontos detectados em frame alterado para o frame original."""
        detection_h, detection_w = detection_frame.shape[:2]
        if detection_w == frame_w and detection_h == frame_h:
            return points

        if detection_w <= 0 or detection_h <= 0 or frame_w <= 0 or frame_h <= 0:
            return points

        try:
            scaled_points = points.astype('float32', copy=True)
            scaled_points[..., 0] *= float(frame_w) / float(detection_w)
            scaled_points[..., 1] *= float(frame_h) / float(detection_h)
            return scaled_points
        except (AttributeError, IndexError, TypeError, ValueError):
            return points

    def _detect_qr_points(self, frame) -> list[tuple[str, object]]:
        """Usa detectAndDecodeMulti quando existe e cai para detector simples."""
        if hasattr(self.detector, 'detectAndDecodeMulti'):
            try:
                result = self.detector.detectAndDecodeMulti(frame)
            except cv2.error as exc:
                self._warn_detector(f'detectAndDecodeMulti falhou: {exc}')
            else:
                multi = self._parse_multi_result(result)
                if multi:
                    return multi

        try:
            data, points, _ = self.detector.detectAndDecode(frame)
        except cv2.error as exc:
            self._warn_detector(f'detectAndDecode falhou: {exc}')
            return []

        if points is None:
            return []

        return [(data or '', points)]

    @staticmethod
    def _parse_multi_result(result) -> list[tuple[str, object]]:
        """Normaliza o retorno variavel de detectAndDecodeMulti."""
        if not isinstance(result, tuple) or len(result) < 3:
            return []

        detected = bool(result[0])
        decoded_info = result[1]
        points = result[2]

        if not detected or points is None:
            return []

        if decoded_info is None:
            decoded_info = [''] * len(points)

        return [
            (data or '', point)
            for data, point in zip(decoded_info, points)
        ]

    def _build_detection(
        self,
        data: str,
        points,
        frame_w: int,
        frame_h: int,
    ) -> Optional[dict]:
        """Converte pontos de QR em um dicionario compativel com vision_node."""
        flat_points = self._flatten_points(points)
        if flat_points is None:
            return None

        decoded = bool(data)
        if not decoded and not self.publish_undecoded:
            return None

        xs = [float(point[0]) for point in flat_points]
        ys = [float(point[1]) for point in flat_points]

        x_min = min(xs)
        x_max = max(xs)
        y_min = min(ys)
        y_max = max(ys)

        box_w = max(0.0, x_max - x_min)
        box_h = max(0.0, y_max - y_min)
        area_ratio = (box_w * box_h) / float(frame_w * frame_h)
        if area_ratio < self.min_area_ratio:
            return None

        cx, cy = compute_bbox_center(x_min, y_min, x_max, y_max)
        error_x, error_y = compute_normalized_error(cx, cy, frame_w, frame_h)

        return {
            'class_id': 0,
            'class_name': 'qr_code',
            'data': data,
            'decoded': decoded,
            'confidence': 1.0,
            'bbox_xyxy': [x_min, y_min, x_max, y_max],
            'center_px': [cx, cy],
            'error_norm': [error_x, error_y],
            'area_ratio': area_ratio,
            'frame_size': [frame_w, frame_h],
        }

    @staticmethod
    def _flatten_points(points):
        """Transforma os pontos retornados pelo OpenCV em lista Nx2."""
        try:
            flat_points = points.reshape(-1, 2)
        except (AttributeError, ValueError):
            return None

        if len(flat_points) < 4:
            return None

        return flat_points

    def _draw_detection(self, annotated, points, detection: dict) -> None:
        """Desenha poligono, centro e texto do QR no frame anotado."""
        flat_points = self._flatten_points(points)
        if flat_points is None:
            return

        try:
            polygon = flat_points.astype('int32').reshape((-1, 1, 2))
            cv2.polylines(annotated, [polygon], True, (0, 255, 0), 2)

            cx, cy = detection['center_px']
            cv2.circle(annotated, (int(cx), int(cy)), 4, (0, 0, 255), -1)

            label = detection['data'] if detection['decoded'] else 'qr_code'
            x_min, y_min, _, _ = detection['bbox_xyxy']
            cv2.putText(
                annotated,
                label,
                (int(x_min), max(0, int(y_min) - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )
        except Exception as exc:
            self._warn_detector(f'Erro ao desenhar QR Code: {exc}')

    def _publish_detections(self, detections: list[dict]) -> None:
        """Publica a lista de QR Codes em JSON."""
        msg = String()
        msg.data = json.dumps(detections, ensure_ascii=False)
        self.detection_pub.publish(msg)

    def _remember_decoded_qr(self, detections: list[dict]) -> None:
        """Guarda o ultimo QR decodificado apenas para debug."""
        for detection in detections:
            if detection.get('decoded') and detection.get('data'):
                self.last_decoded_value = detection['data']
                self.last_qr_seen_time_ns = self.get_clock().now().nanoseconds
                return

    def _get_last_seen_age(self) -> Optional[float]:
        """Retorna ha quantos segundos o ultimo QR decodificado foi visto."""
        if self.last_qr_seen_time_ns <= 0:
            return None

        age = (
            self.get_clock().now().nanoseconds - self.last_qr_seen_time_ns
        ) / 1e9
        return max(0.0, age)

    def _publish_debug(
        self,
        detections: list[dict],
        detection_method: Optional[str],
    ) -> None:
        """Publica contagem, metodo usado e ultimo QR visto para depuracao."""
        decoded_values = [
            detection['data']
            for detection in detections
            if detection.get('decoded')
        ]

        msg = String()
        msg.data = json.dumps({
            'qr_count': len(detections),
            'decoded_values': decoded_values,
            'detection_method': detection_method,
            'last_decoded_value': self.last_decoded_value,
            'last_seen_age': self._get_last_seen_age(),
        }, ensure_ascii=False)
        self.debug_pub.publish(msg)

    def _publish_annotated_image(self, annotated, header) -> None:
        """Publica o frame anotado preservando o header da imagem original."""
        try:
            out_img = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            out_img.header = header
            self.image_pub.publish(out_img)
        except Exception as exc:
            self.get_logger().error(f'Erro ao publicar imagem anotada QR: {exc}')

    def _warn_detector(self, message: str) -> None:
        """Emite avisos do detector com throttle de tempo."""
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_detector_warning_time_ns < int(2.0 * 1e9):
            return

        self.get_logger().warn(message)
        self.last_detector_warning_time_ns = now_ns

    def destroy_node(self):
        """Fecha a janela de preview do QR antes do shutdown."""
        if self.show_preview:
            try:
                cv2.destroyWindow('QR Preview')
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    """Start qr_node and publish QR detections from camera frames."""
    rclpy.init(args=args)
    node = QRNode()

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

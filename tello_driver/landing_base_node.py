#!/usr/bin/env python3

import json
import math
from typing import Optional

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from tello_driver.visual_math import (
    clamp,
    compute_bbox_center,
    compute_normalized_error,
)


class LandingBaseNode(Node):
    """Detecta bases azul/amarelo usando cor, geometria e padrao interno."""

    def __init__(self):
        """Declara parametros, prepara publishers/subscriber e memoria temporal."""
        super().__init__('landing_base_node')

        self.declare_parameter('input_topic', '/tello/image_raw')
        self.declare_parameter('output_detection_topic', '/vision/landing_base')
        self.declare_parameter(
            'output_image_topic',
            '/vision/landing_base_image_annotated',
        )
        self.declare_parameter('debug_topic', '/vision/landing_base_debug')
        self.declare_parameter('output_mask_topic', '/vision/landing_base_mask')
        self.declare_parameter('show_preview', False)
        self.declare_parameter('process_every_n_frames', 1)

        self.declare_parameter('min_area_ratio', 0.01)
        self.declare_parameter('max_area_ratio', 0.80)
        self.declare_parameter('min_yellow_ratio_in_bbox', 0.02)
        self.declare_parameter('min_blue_ratio_in_bbox', 0.05)
        self.declare_parameter('min_color_score', 0.45)
        self.declare_parameter('min_shape_score', 0.35)
        self.declare_parameter('min_pattern_score', 0.35)
        self.declare_parameter('max_color_only_confidence', 0.60)
        self.declare_parameter('max_overexposed_ratio', 0.35)

        self.declare_parameter('landing_square_max_aspect', 1.60)
        self.declare_parameter('enable_takeoff_base_detection', True)
        self.declare_parameter('takeoff_min_aspect', 1.60)
        self.declare_parameter('takeoff_max_aspect', 4.00)

        self.declare_parameter('pattern_roi_size', 240)
        self.declare_parameter('border_band_fraction', 0.08)
        self.declare_parameter('circle_radius_min_fraction', 0.21)
        self.declare_parameter('circle_radius_max_fraction', 0.34)
        self.declare_parameter('cross_band_fraction', 0.06)

        self.declare_parameter('temporal_window_size', 5)
        self.declare_parameter('min_temporal_hits', 2)
        self.declare_parameter('temporal_max_center_distance_norm', 0.18)
        self.declare_parameter('temporal_max_area_ratio_delta', 0.12)

        self.declare_parameter('morph_kernel_size', 5)
        self.declare_parameter('publish_empty', True)
        self.declare_parameter('publish_mask', False)

        self.declare_parameter('yellow_lower_h', 20)
        self.declare_parameter('yellow_lower_s', 80)
        self.declare_parameter('yellow_lower_v', 80)
        self.declare_parameter('yellow_upper_h', 40)
        self.declare_parameter('yellow_upper_s', 255)
        self.declare_parameter('yellow_upper_v', 255)

        self.declare_parameter('blue_lower_h', 90)
        self.declare_parameter('blue_lower_s', 60)
        self.declare_parameter('blue_lower_v', 50)
        self.declare_parameter('blue_upper_h', 135)
        self.declare_parameter('blue_upper_s', 255)
        self.declare_parameter('blue_upper_v', 255)

        self.input_topic = self.get_parameter('input_topic').value
        self.output_detection_topic = self.get_parameter(
            'output_detection_topic',
        ).value
        self.output_image_topic = self.get_parameter('output_image_topic').value
        self.debug_topic = self.get_parameter('debug_topic').value
        self.output_mask_topic = self.get_parameter('output_mask_topic').value
        self.show_preview = bool(self.get_parameter('show_preview').value)
        self.process_every_n_frames = max(
            1,
            int(self.get_parameter('process_every_n_frames').value),
        )

        self.min_area_ratio = float(self.get_parameter('min_area_ratio').value)
        self.max_area_ratio = float(self.get_parameter('max_area_ratio').value)
        self.min_yellow_ratio_in_bbox = float(
            self.get_parameter('min_yellow_ratio_in_bbox').value,
        )
        self.min_blue_ratio_in_bbox = float(
            self.get_parameter('min_blue_ratio_in_bbox').value,
        )
        self.min_color_score = float(self.get_parameter('min_color_score').value)
        self.min_shape_score = float(self.get_parameter('min_shape_score').value)
        self.min_pattern_score = float(self.get_parameter('min_pattern_score').value)
        self.max_color_only_confidence = float(
            self.get_parameter('max_color_only_confidence').value,
        )
        self.max_overexposed_ratio = float(
            self.get_parameter('max_overexposed_ratio').value,
        )
        self.landing_square_max_aspect = max(
            1.01,
            float(self.get_parameter('landing_square_max_aspect').value),
        )
        self.enable_takeoff_base_detection = bool(
            self.get_parameter('enable_takeoff_base_detection').value,
        )
        self.takeoff_min_aspect = max(
            1.01,
            float(self.get_parameter('takeoff_min_aspect').value),
        )
        self.takeoff_max_aspect = max(
            self.takeoff_min_aspect,
            float(self.get_parameter('takeoff_max_aspect').value),
        )
        self.pattern_roi_size = max(
            40,
            int(self.get_parameter('pattern_roi_size').value),
        )
        self.border_band_fraction = float(
            clamp(self.get_parameter('border_band_fraction').value, 0.01, 0.25),
        )
        self.circle_radius_min_fraction = float(
            clamp(
                self.get_parameter('circle_radius_min_fraction').value,
                0.05,
                0.45,
            ),
        )
        self.circle_radius_max_fraction = float(
            clamp(
                self.get_parameter('circle_radius_max_fraction').value,
                self.circle_radius_min_fraction,
                0.50,
            ),
        )
        self.cross_band_fraction = float(
            clamp(self.get_parameter('cross_band_fraction').value, 0.01, 0.20),
        )
        self.temporal_window_size = max(
            1,
            int(self.get_parameter('temporal_window_size').value),
        )
        self.min_temporal_hits = max(
            1,
            int(self.get_parameter('min_temporal_hits').value),
        )
        self.temporal_max_center_distance_norm = float(
            self.get_parameter('temporal_max_center_distance_norm').value,
        )
        self.temporal_max_area_ratio_delta = float(
            self.get_parameter('temporal_max_area_ratio_delta').value,
        )
        self.morph_kernel_size = max(
            1,
            int(self.get_parameter('morph_kernel_size').value),
        )
        self.publish_empty = bool(self.get_parameter('publish_empty').value)
        self.publish_mask = bool(self.get_parameter('publish_mask').value)
        self.yellow_lower = self._read_hsv_bound('yellow_lower')
        self.yellow_upper = self._read_hsv_bound('yellow_upper')
        self.blue_lower = self._read_hsv_bound('blue_lower')
        self.blue_upper = self._read_hsv_bound('blue_upper')

        self.bridge = CvBridge()
        self.frame_count = 0
        self.recent_candidates = []
        self.last_detector_warning_time_ns = 0

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
        self.mask_pub = None
        if self.publish_mask:
            self.mask_pub = self.create_publisher(
                Image,
                self.output_mask_topic,
                10,
            )

        self.get_logger().info('Landing base node iniciado')
        self.get_logger().info(f'Assinando imagem em: {self.input_topic}')
        self.get_logger().info(
            f'Publicando bases em: {self.output_detection_topic}',
        )

    def _image_callback(self, msg: Image) -> None:
        """Recebe frames da camera e publica deteccao, debug e imagens auxiliares."""
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

        detections, annotated, debug = self._detect_and_annotate(frame)

        if detections or self.publish_empty:
            self._publish_detections(detections)

        self._publish_debug(debug, detections)
        self._publish_annotated_image(annotated, msg.header)
        if self.publish_mask:
            self._publish_mask(debug['combined_mask'], msg.header)

        if self.show_preview:
            cv2.imshow('Landing Base Preview', annotated)
            cv2.waitKey(1)

    def _detect_and_annotate(self, frame):
        """Executa o pipeline completo de deteccao e retorna JSON + anotacao."""
        annotated = frame.copy()
        frame_h, frame_w = frame.shape[:2]

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        yellow_mask = cv2.inRange(hsv, self.yellow_lower, self.yellow_upper)
        blue_mask = cv2.inRange(hsv, self.blue_lower, self.blue_upper)

        yellow_mask = self._clean_mask(yellow_mask)
        blue_mask = self._clean_mask(blue_mask)
        combined_mask = cv2.bitwise_or(yellow_mask, blue_mask)
        combined_mask = self._clean_mask(combined_mask)

        contours = self._find_contours(combined_mask)
        candidates = []
        valid_candidates = []

        for contour in contours:
            candidate = self._build_candidate(
                contour,
                frame,
                yellow_mask,
                blue_mask,
                frame_w,
                frame_h,
            )
            if candidate is None:
                continue

            candidates.append(candidate)
            if candidate['static_valid']:
                valid_candidates.append(candidate)

        selected = self._select_candidate(valid_candidates)
        detections = []
        if selected is not None:
            self._apply_temporal_score(selected, frame_w, frame_h)
            selected['confidence'] = self._compute_confidence(selected)
            selected['valid'] = (
                selected['static_valid']
                and selected['temporal_hits'] >= self.min_temporal_hits
            )

            if selected['valid']:
                detection = self._build_detection(selected, frame_w, frame_h)
                detections.append(detection)
                self._draw_detection(annotated, detection)

        self._draw_camera_center(annotated)

        debug = self._build_debug(
            detections,
            selected,
            candidates,
            valid_candidates,
            contours,
            yellow_mask,
            blue_mask,
            combined_mask,
            frame_w,
            frame_h,
        )
        debug['combined_mask'] = combined_mask

        return detections, annotated, debug

    def _read_hsv_bound(self, prefix: str) -> tuple[int, int, int]:
        """Le um limite HSV dos parametros e prende os valores na faixa valida."""
        h = int(self.get_parameter(f'{prefix}_h').value)
        s = int(self.get_parameter(f'{prefix}_s').value)
        v = int(self.get_parameter(f'{prefix}_v').value)
        return (
            int(clamp(h, 0, 179)),
            int(clamp(s, 0, 255)),
            int(clamp(v, 0, 255)),
        )

    def _clean_mask(self, mask):
        """Aplica abertura/fechamento morfologico para reduzir ruido."""
        if self.morph_kernel_size <= 1:
            return mask

        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (self.morph_kernel_size, self.morph_kernel_size),
        )
        opened = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        return cv2.morphologyEx(opened, cv2.MORPH_CLOSE, kernel)

    @staticmethod
    def _find_contours(mask):
        """Extrai contornos externos da mascara combinada azul/amarelo."""
        contours_result = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )
        return contours_result[-2]

    def _build_candidate(
        self,
        contour,
        frame,
        yellow_mask,
        blue_mask,
        frame_w: int,
        frame_h: int,
    ) -> Optional[dict]:
        """Calcula todos os scores de um contorno candidato a base."""
        x, y, w, h = cv2.boundingRect(contour)
        if w <= 0 or h <= 0 or frame_w <= 0 or frame_h <= 0:
            return None

        bbox_area = float(w * h)
        frame_area = float(frame_w * frame_h)
        area_ratio = bbox_area / frame_area
        yellow_ratio = self._mask_ratio(yellow_mask, x, y, w, h)
        blue_ratio = self._mask_ratio(blue_mask, x, y, w, h)
        color_score = self._compute_color_score(
            area_ratio,
            yellow_ratio,
            blue_ratio,
        )
        shape = self._score_shape(contour, w, h, bbox_area)
        pattern = self._score_pattern(yellow_mask, blue_mask, x, y, w, h)
        overexposed_ratio = self._overexposed_ratio(frame, x, y, w, h)
        exposure_score = 1.0 - self._threshold_score(
            overexposed_ratio,
            self.max_overexposed_ratio,
        )

        static_valid = (
            self.min_area_ratio <= area_ratio <= self.max_area_ratio
            and yellow_ratio >= self.min_yellow_ratio_in_bbox
            and blue_ratio >= self.min_blue_ratio_in_bbox
            and color_score >= self.min_color_score
            and shape['shape_score'] >= self.min_shape_score
            and pattern['pattern_score'] >= self.min_pattern_score
            and overexposed_ratio <= self.max_overexposed_ratio
        )

        cx, cy = compute_bbox_center(x, y, x + w, y + h)
        confidence = self._compute_confidence({
            'color_score': color_score,
            'shape_score': shape['shape_score'],
            'pattern_score': pattern['pattern_score'],
            'exposure_score': exposure_score,
            'temporal_score': 0.0,
        })

        return {
            'bbox_xyxy': [float(x), float(y), float(x + w), float(y + h)],
            'area_ratio': area_ratio,
            'aspect_ratio': shape['aspect_ratio'],
            'rectangularity': shape['rectangularity'],
            'class_name': shape['class_name'],
            'yellow_ratio_in_bbox': yellow_ratio,
            'blue_ratio_in_bbox': blue_ratio,
            'color_score': color_score,
            'shape_score': shape['shape_score'],
            'pattern_score': pattern['pattern_score'],
            'border_score': pattern['border_score'],
            'center_blue_score': pattern['center_blue_score'],
            'circle_score': pattern['circle_score'],
            'cross_score': pattern['cross_score'],
            'overexposed_ratio': overexposed_ratio,
            'exposure_score': exposure_score,
            'temporal_hits': 0,
            'temporal_score': 0.0,
            'center_px': [cx, cy],
            'confidence': confidence,
            'static_valid': static_valid,
            'valid': False,
        }

    @staticmethod
    def _mask_ratio(mask, x: int, y: int, w: int, h: int) -> float:
        """Calcula a fracao de pixels ativos dentro de um bbox."""
        crop = mask[y:y + h, x:x + w]
        if crop.size <= 0 or w <= 0 or h <= 0:
            return 0.0
        return float(cv2.countNonZero(crop)) / float(w * h)

    def _compute_color_score(
        self,
        area_ratio: float,
        yellow_ratio: float,
        blue_ratio: float,
    ) -> float:
        """Combina area e proporcoes de amarelo/azul em um score de cor."""
        area_score = self._threshold_score(area_ratio, self.min_area_ratio)
        yellow_score = self._threshold_score(
            yellow_ratio,
            self.min_yellow_ratio_in_bbox,
        )
        blue_score = self._threshold_score(
            blue_ratio,
            self.min_blue_ratio_in_bbox,
        )

        color_score = (
            0.10 * area_score
            + 0.45 * yellow_score
            + 0.45 * blue_score
        )
        return float(clamp(color_score, 0.0, 1.0))

    def _score_shape(self, contour, w: int, h: int, bbox_area: float) -> dict:
        """Classifica formato como base quadrada ou retangular de takeoff."""
        contour_area = max(0.0, float(cv2.contourArea(contour)))
        rectangularity = 0.0
        if bbox_area > 0.0:
            rectangularity = float(clamp(contour_area / bbox_area, 0.0, 1.0))

        raw_aspect = float(w) / float(h) if h > 0 else 0.0
        aspect_ratio = max(raw_aspect, 1.0 / raw_aspect) if raw_aspect > 0 else 0.0

        landing_aspect_score = 1.0 - clamp(
            (aspect_ratio - 1.0) / (self.landing_square_max_aspect - 1.0),
            0.0,
            1.0,
        )
        landing_score = rectangularity * landing_aspect_score

        takeoff_score = 0.0
        if (
            self.enable_takeoff_base_detection
            and self.takeoff_min_aspect <= aspect_ratio <= self.takeoff_max_aspect
        ):
            takeoff_score = rectangularity

        if takeoff_score > landing_score:
            return {
                'class_name': 'takeoff_base',
                'shape_score': float(takeoff_score),
                'aspect_ratio': aspect_ratio,
                'rectangularity': rectangularity,
            }

        return {
            'class_name': 'landing_base',
            'shape_score': float(landing_score),
            'aspect_ratio': aspect_ratio,
            'rectangularity': rectangularity,
        }

    def _score_pattern(
        self,
        yellow_mask,
        blue_mask,
        x: int,
        y: int,
        w: int,
        h: int,
    ) -> dict:
        """Mede borda, centro azul, circulo e cruz no ROI normalizado."""
        yellow_crop = yellow_mask[y:y + h, x:x + w]
        blue_crop = blue_mask[y:y + h, x:x + w]
        if yellow_crop.size <= 0 or blue_crop.size <= 0:
            return self._empty_pattern_score()

        size = self.pattern_roi_size
        yellow_roi = cv2.resize(
            yellow_crop,
            (size, size),
            interpolation=cv2.INTER_NEAREST,
        ) > 0
        blue_roi = cv2.resize(
            blue_crop,
            (size, size),
            interpolation=cv2.INTER_NEAREST,
        ) > 0

        band = max(2, int(size * self.border_band_fraction))
        border_mask = np.zeros((size, size), dtype=bool)
        border_mask[:band, :] = True
        border_mask[-band:, :] = True
        border_mask[:, :band] = True
        border_mask[:, -band:] = True
        border_ratio = self._boolean_ratio(yellow_roi, border_mask)
        border_score = self._threshold_score(border_ratio, 0.25)

        center_margin = int(size * 0.18)
        center_mask = np.zeros((size, size), dtype=bool)
        center_mask[
            center_margin:size - center_margin,
            center_margin:size - center_margin,
        ] = True
        center_blue_ratio = self._boolean_ratio(blue_roi, center_mask)
        center_blue_score = self._threshold_score(center_blue_ratio, 0.45)

        yy, xx = np.ogrid[:size, :size]
        center = (float(size - 1) / 2.0, float(size - 1) / 2.0)
        distance = np.sqrt((xx - center[0]) ** 2 + (yy - center[1]) ** 2)
        radius_min = size * self.circle_radius_min_fraction
        radius_max = size * self.circle_radius_max_fraction
        circle_mask = (distance >= radius_min) & (distance <= radius_max)
        circle_ratio = self._boolean_ratio(yellow_roi, circle_mask)
        circle_score = self._threshold_score(circle_ratio, 0.20)

        cross_band = max(2, int(size * self.cross_band_fraction))
        cross_start = int(size * 0.28)
        cross_end = int(size * 0.72)
        center_idx = size // 2
        horizontal_mask = np.zeros((size, size), dtype=bool)
        vertical_mask = np.zeros((size, size), dtype=bool)
        horizontal_mask[
            max(0, center_idx - cross_band):min(size, center_idx + cross_band),
            cross_start:cross_end,
        ] = True
        vertical_mask[
            cross_start:cross_end,
            max(0, center_idx - cross_band):min(size, center_idx + cross_band),
        ] = True
        horizontal_ratio = self._boolean_ratio(yellow_roi, horizontal_mask)
        vertical_ratio = self._boolean_ratio(yellow_roi, vertical_mask)
        cross_ratio = (horizontal_ratio + vertical_ratio) / 2.0
        cross_score = self._threshold_score(cross_ratio, 0.25)

        pattern_score = (
            0.20 * border_score
            + 0.20 * center_blue_score
            + 0.30 * circle_score
            + 0.30 * cross_score
        )

        return {
            'pattern_score': float(clamp(pattern_score, 0.0, 1.0)),
            'border_score': float(clamp(border_score, 0.0, 1.0)),
            'center_blue_score': float(clamp(center_blue_score, 0.0, 1.0)),
            'circle_score': float(clamp(circle_score, 0.0, 1.0)),
            'cross_score': float(clamp(cross_score, 0.0, 1.0)),
        }

    @staticmethod
    def _empty_pattern_score() -> dict:
        """Retorna scores de padrao zerados para candidatos invalidos."""
        return {
            'pattern_score': 0.0,
            'border_score': 0.0,
            'center_blue_score': 0.0,
            'circle_score': 0.0,
            'cross_score': 0.0,
        }

    @staticmethod
    def _boolean_ratio(values, mask) -> float:
        """Conta a fracao de pixels True dentro de uma mascara booleana."""
        total = int(np.count_nonzero(mask))
        if total <= 0:
            return 0.0
        return float(np.count_nonzero(values & mask)) / float(total)

    @staticmethod
    def _overexposed_ratio(frame, x: int, y: int, w: int, h: int) -> float:
        """Estima quanto do candidato esta branco/saturado por brilho alto."""
        crop = frame[y:y + h, x:x + w]
        if crop.size <= 0:
            return 0.0

        hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
        saturation = hsv[:, :, 1]
        value = hsv[:, :, 2]
        overexposed = (value >= 245) & (saturation <= 40)
        return float(np.count_nonzero(overexposed)) / float(w * h)

    def _compute_confidence(self, candidate: dict) -> float:
        """Combina scores de cor, forma, padrao, exposicao e tempo."""
        confidence = (
            0.25 * candidate.get('color_score', 0.0)
            + 0.20 * candidate.get('shape_score', 0.0)
            + 0.35 * candidate.get('pattern_score', 0.0)
            + 0.10 * candidate.get('exposure_score', 0.0)
            + 0.10 * candidate.get('temporal_score', 0.0)
        )

        if (
            candidate.get('shape_score', 0.0) < self.min_shape_score
            or candidate.get('pattern_score', 0.0) < self.min_pattern_score
        ):
            confidence = min(confidence, self.max_color_only_confidence)

        return float(clamp(confidence, 0.0, 1.0))

    @staticmethod
    def _threshold_score(value: float, threshold: float) -> float:
        """Normaliza um valor em relacao a um limiar esperado."""
        if threshold <= 0.0:
            return 1.0
        return float(clamp(value / (2.0 * threshold), 0.0, 1.0))

    def _apply_temporal_score(
        self,
        candidate: dict,
        frame_w: int,
        frame_h: int,
    ) -> None:
        """Atualiza estabilidade temporal comparando com candidatos recentes."""
        if not hasattr(self, 'recent_candidates'):
            self.recent_candidates = []

        cx, cy = candidate['center_px']
        center_norm = [
            float(cx) / float(max(1, frame_w)),
            float(cy) / float(max(1, frame_h)),
        ]

        hits = 1
        for previous in self.recent_candidates:
            if previous['class_name'] != candidate['class_name']:
                continue

            dx = center_norm[0] - previous['center_norm'][0]
            dy = center_norm[1] - previous['center_norm'][1]
            distance = math.sqrt(dx * dx + dy * dy)
            area_delta = abs(
                candidate['area_ratio'] - previous['area_ratio'],
            )

            if (
                distance <= self.temporal_max_center_distance_norm
                and area_delta <= self.temporal_max_area_ratio_delta
            ):
                hits += 1

        candidate['temporal_hits'] = min(hits, self.temporal_window_size)
        candidate['temporal_score'] = float(
            clamp(
                candidate['temporal_hits'] / float(self.min_temporal_hits),
                0.0,
                1.0,
            ),
        )

        self.recent_candidates.append({
            'class_name': candidate['class_name'],
            'center_norm': center_norm,
            'area_ratio': candidate['area_ratio'],
        })
        if len(self.recent_candidates) > self.temporal_window_size:
            self.recent_candidates = self.recent_candidates[
                -self.temporal_window_size:
            ]

    @staticmethod
    def _select_candidate(candidates: list[dict]) -> Optional[dict]:
        """Escolhe o melhor candidato por score composto e area."""
        if not candidates:
            return None
        return max(
            candidates,
            key=lambda candidate: (
                candidate['color_score']
                + candidate['shape_score']
                + candidate['pattern_score'],
                candidate['area_ratio'],
            ),
        )

    def _build_detection(self, candidate: dict, frame_w: int, frame_h: int) -> dict:
        """Monta o dicionario publicado no mesmo contrato do vision_node."""
        x1, y1, x2, y2 = candidate['bbox_xyxy']
        cx, cy = compute_bbox_center(x1, y1, x2, y2)
        error_x, error_y = compute_normalized_error(cx, cy, frame_w, frame_h)

        return {
            'class_id': -1,
            'class_name': candidate['class_name'],
            'confidence': candidate['confidence'],
            'bbox_xyxy': [x1, y1, x2, y2],
            'area_ratio': candidate['area_ratio'],
            'aspect_ratio': candidate['aspect_ratio'],
            'rectangularity': candidate['rectangularity'],
            'center_px': [cx, cy],
            'error_norm': [error_x, error_y],
            'frame_size': [frame_w, frame_h],
            'yellow_ratio_in_bbox': candidate['yellow_ratio_in_bbox'],
            'blue_ratio_in_bbox': candidate['blue_ratio_in_bbox'],
            'color_score': candidate['color_score'],
            'shape_score': candidate['shape_score'],
            'pattern_score': candidate['pattern_score'],
            'border_score': candidate['border_score'],
            'center_blue_score': candidate['center_blue_score'],
            'circle_score': candidate['circle_score'],
            'cross_score': candidate['cross_score'],
            'temporal_hits': candidate['temporal_hits'],
            'temporal_score': candidate['temporal_score'],
            'overexposed_ratio': candidate['overexposed_ratio'],
            'exposure_score': candidate['exposure_score'],
        }

    def _draw_detection(self, annotated, detection: dict) -> None:
        """Desenha bbox, centro, linha ate o centro e scores principais."""
        x1, y1, x2, y2 = detection['bbox_xyxy']
        cx, cy = detection['center_px']
        error_x, _ = detection['error_norm']

        p1 = (int(x1), int(y1))
        p2 = (int(x2), int(y2))
        center = (int(cx), int(cy))
        frame_h, frame_w = annotated.shape[:2]
        camera_center = (frame_w // 2, frame_h // 2)

        cv2.rectangle(annotated, p1, p2, (0, 255, 0), 2)
        cv2.circle(annotated, center, 5, (0, 0, 255), -1)
        cv2.line(annotated, camera_center, center, (0, 255, 255), 2)

        label = (
            f"{detection['class_name']} conf={detection['confidence']:.2f} "
            f"area={detection['area_ratio']:.3f} ex={error_x:.2f}"
        )
        color_label = (
            f"color={detection['color_score']:.2f} "
            f"shape={detection['shape_score']:.2f} "
            f"pattern={detection['pattern_score']:.2f}"
        )

        text_x = max(0, int(x1))
        text_y = max(18, int(y1) - 8)
        cv2.putText(
            annotated,
            label,
            (text_x, text_y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            annotated,
            color_label,
            (text_x, min(frame_h - 8, text_y + 18)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (0, 255, 255),
            1,
            cv2.LINE_AA,
        )

    @staticmethod
    def _draw_camera_center(annotated) -> None:
        """Desenha cruz no centro da imagem para referencia visual."""
        frame_h, frame_w = annotated.shape[:2]
        center_x = frame_w // 2
        center_y = frame_h // 2

        cv2.line(annotated, (center_x, 0), (center_x, frame_h), (255, 255, 255), 1)
        cv2.line(annotated, (0, center_y), (frame_w, center_y), (255, 255, 255), 1)

    def _build_debug(
        self,
        detections: list[dict],
        selected: Optional[dict],
        candidates: list[dict],
        valid_candidates: list[dict],
        contours,
        yellow_mask,
        blue_mask,
        combined_mask,
        frame_w: int,
        frame_h: int,
    ) -> dict:
        """Monta payload de debug com contagens, scores e thresholds."""
        frame_area = max(1.0, float(frame_w * frame_h))
        top_candidates = sorted(
            candidates,
            key=lambda candidate: candidate['area_ratio'],
            reverse=True,
        )[:5]

        return {
            'landing_base_count': len(detections),
            'detection_class_counts': self._class_counts(detections),
            'contour_count': len(contours),
            'candidate_count': len(candidates),
            'static_valid_candidate_count': len(valid_candidates),
            'valid_candidate_count': len([
                candidate for candidate in candidates if candidate['valid']
            ]),
            'selected_candidate': self._candidate_summary(selected),
            'top_candidates': [
                self._candidate_summary(candidate)
                for candidate in top_candidates
            ],
            'yellow_mask_ratio': (
                float(cv2.countNonZero(yellow_mask)) / frame_area
            ),
            'blue_mask_ratio': float(cv2.countNonZero(blue_mask)) / frame_area,
            'combined_mask_ratio': (
                float(cv2.countNonZero(combined_mask)) / frame_area
            ),
            'thresholds': {
                'min_area_ratio': self.min_area_ratio,
                'max_area_ratio': self.max_area_ratio,
                'min_yellow_ratio_in_bbox': self.min_yellow_ratio_in_bbox,
                'min_blue_ratio_in_bbox': self.min_blue_ratio_in_bbox,
                'min_color_score': self.min_color_score,
                'min_shape_score': self.min_shape_score,
                'min_pattern_score': self.min_pattern_score,
                'max_overexposed_ratio': self.max_overexposed_ratio,
                'landing_square_max_aspect': self.landing_square_max_aspect,
                'enable_takeoff_base_detection': self.enable_takeoff_base_detection,
                'takeoff_min_aspect': self.takeoff_min_aspect,
                'takeoff_max_aspect': self.takeoff_max_aspect,
                'temporal_window_size': self.temporal_window_size,
                'min_temporal_hits': self.min_temporal_hits,
                'morph_kernel_size': self.morph_kernel_size,
                'yellow_lower_hsv': list(self.yellow_lower),
                'yellow_upper_hsv': list(self.yellow_upper),
                'blue_lower_hsv': list(self.blue_lower),
                'blue_upper_hsv': list(self.blue_upper),
            },
        }

    @staticmethod
    def _class_counts(detections: list[dict]) -> dict:
        """Conta quantas deteccoes existem por classe publicada."""
        counts = {}
        for detection in detections:
            class_name = detection.get('class_name', 'unknown')
            counts[class_name] = counts.get(class_name, 0) + 1
        return counts

    @staticmethod
    def _candidate_summary(candidate: Optional[dict]) -> Optional[dict]:
        """Reduz um candidato aos campos uteis para debug JSON."""
        if candidate is None:
            return None

        return {
            'bbox_xyxy': candidate['bbox_xyxy'],
            'class_name': candidate['class_name'],
            'area_ratio': candidate['area_ratio'],
            'aspect_ratio': candidate['aspect_ratio'],
            'rectangularity': candidate['rectangularity'],
            'yellow_ratio_in_bbox': candidate['yellow_ratio_in_bbox'],
            'blue_ratio_in_bbox': candidate['blue_ratio_in_bbox'],
            'color_score': candidate['color_score'],
            'shape_score': candidate['shape_score'],
            'pattern_score': candidate['pattern_score'],
            'border_score': candidate['border_score'],
            'center_blue_score': candidate['center_blue_score'],
            'circle_score': candidate['circle_score'],
            'cross_score': candidate['cross_score'],
            'temporal_hits': candidate['temporal_hits'],
            'temporal_score': candidate['temporal_score'],
            'overexposed_ratio': candidate['overexposed_ratio'],
            'exposure_score': candidate['exposure_score'],
            'confidence': candidate['confidence'],
            'static_valid': candidate['static_valid'],
            'valid': candidate['valid'],
        }

    def _publish_detections(self, detections: list[dict]) -> None:
        """Publica lista de deteccoes em JSON."""
        msg = String()
        msg.data = json.dumps(detections, ensure_ascii=False)
        self.detection_pub.publish(msg)

    def _publish_debug(self, debug: dict, detections: list[dict]) -> None:
        """Publica debug sem incluir a mascara binaria bruta."""
        msg = String()
        serializable_debug = {
            key: value
            for key, value in debug.items()
            if key != 'combined_mask'
        }
        msg.data = json.dumps({
            'detections': detections,
            **serializable_debug,
        }, ensure_ascii=False)
        self.debug_pub.publish(msg)

    def _publish_annotated_image(self, annotated, header) -> None:
        """Publica frame anotado preservando o header original."""
        try:
            out_img = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            out_img.header = header
            self.image_pub.publish(out_img)
        except Exception as exc:
            self.get_logger().error(
                f'Erro ao publicar imagem anotada da base: {exc}',
            )

    def _publish_mask(self, mask, header) -> None:
        """Publica a mascara combinada azul/amarelo quando habilitada."""
        if self.mask_pub is None:
            return

        try:
            mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
            out_img = self.bridge.cv2_to_imgmsg(mask_bgr, encoding='bgr8')
            out_img.header = header
            self.mask_pub.publish(out_img)
        except Exception as exc:
            self.get_logger().error(
                f'Erro ao publicar mascara da base: {exc}',
            )

    def _warn_detector(self, message: str) -> None:
        """Loga avisos do detector com throttle de tempo."""
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_detector_warning_time_ns < int(2.0 * 1e9):
            return

        self.get_logger().warn(message)
        self.last_detector_warning_time_ns = now_ns

    def destroy_node(self):
        """Fecha janela OpenCV opcional antes do shutdown."""
        if self.show_preview:
            try:
                cv2.destroyWindow('Landing Base Preview')
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    """Start landing_base_node and publish base detections from camera frames."""
    rclpy.init(args=args)
    node = LandingBaseNode()

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

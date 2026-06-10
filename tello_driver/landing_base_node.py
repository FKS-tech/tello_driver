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

from tello_driver.visual_math import (
    clamp,
    compute_bbox_center,
    compute_normalized_error,
)


class LandingBaseNode(Node):
    def __init__(self):
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
                yellow_mask,
                blue_mask,
                frame_w,
                frame_h,
            )
            if candidate is None:
                continue

            candidates.append(candidate)
            if candidate['valid']:
                valid_candidates.append(candidate)

        selected = self._select_candidate(valid_candidates)
        detections = []
        if selected is not None:
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
        h = int(self.get_parameter(f'{prefix}_h').value)
        s = int(self.get_parameter(f'{prefix}_s').value)
        v = int(self.get_parameter(f'{prefix}_v').value)
        return (
            int(clamp(h, 0, 179)),
            int(clamp(s, 0, 255)),
            int(clamp(v, 0, 255)),
        )

    def _clean_mask(self, mask):
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
        contours_result = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )
        return contours_result[-2]

    def _build_candidate(
        self,
        contour,
        yellow_mask,
        blue_mask,
        frame_w: int,
        frame_h: int,
    ) -> Optional[dict]:
        x, y, w, h = cv2.boundingRect(contour)
        if w <= 0 or h <= 0 or frame_w <= 0 or frame_h <= 0:
            return None

        bbox_area = float(w * h)
        frame_area = float(frame_w * frame_h)
        area_ratio = bbox_area / frame_area
        yellow_ratio = self._mask_ratio(yellow_mask, x, y, w, h)
        blue_ratio = self._mask_ratio(blue_mask, x, y, w, h)

        valid = (
            self.min_area_ratio <= area_ratio <= self.max_area_ratio
            and yellow_ratio >= self.min_yellow_ratio_in_bbox
            and blue_ratio >= self.min_blue_ratio_in_bbox
        )

        confidence = self._compute_confidence(
            area_ratio,
            yellow_ratio,
            blue_ratio,
        )

        return {
            'bbox_xyxy': [float(x), float(y), float(x + w), float(y + h)],
            'area_ratio': area_ratio,
            'yellow_ratio_in_bbox': yellow_ratio,
            'blue_ratio_in_bbox': blue_ratio,
            'confidence': confidence,
            'valid': valid,
        }

    @staticmethod
    def _mask_ratio(mask, x: int, y: int, w: int, h: int) -> float:
        crop = mask[y:y + h, x:x + w]
        if crop.size <= 0 or w <= 0 or h <= 0:
            return 0.0
        return float(cv2.countNonZero(crop)) / float(w * h)

    def _compute_confidence(
        self,
        area_ratio: float,
        yellow_ratio: float,
        blue_ratio: float,
    ) -> float:
        area_score = self._threshold_score(area_ratio, self.min_area_ratio)
        yellow_score = self._threshold_score(
            yellow_ratio,
            self.min_yellow_ratio_in_bbox,
        )
        blue_score = self._threshold_score(
            blue_ratio,
            self.min_blue_ratio_in_bbox,
        )

        confidence = (
            0.15 * area_score
            + 0.40 * yellow_score
            + 0.45 * blue_score
        )
        return float(clamp(confidence, 0.0, 1.0))

    @staticmethod
    def _threshold_score(value: float, threshold: float) -> float:
        if threshold <= 0.0:
            return 1.0
        return float(clamp(value / (2.0 * threshold), 0.0, 1.0))

    @staticmethod
    def _select_candidate(candidates: list[dict]) -> Optional[dict]:
        if not candidates:
            return None
        return max(candidates, key=lambda candidate: candidate['area_ratio'])

    def _build_detection(self, candidate: dict, frame_w: int, frame_h: int) -> dict:
        x1, y1, x2, y2 = candidate['bbox_xyxy']
        cx, cy = compute_bbox_center(x1, y1, x2, y2)
        error_x, error_y = compute_normalized_error(cx, cy, frame_w, frame_h)

        return {
            'class_id': -1,
            'class_name': 'landing_base',
            'confidence': candidate['confidence'],
            'bbox_xyxy': [x1, y1, x2, y2],
            'area_ratio': candidate['area_ratio'],
            'center_px': [cx, cy],
            'error_norm': [error_x, error_y],
            'frame_size': [frame_w, frame_h],
            'yellow_ratio_in_bbox': candidate['yellow_ratio_in_bbox'],
            'blue_ratio_in_bbox': candidate['blue_ratio_in_bbox'],
        }

    def _draw_detection(self, annotated, detection: dict) -> None:
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
            f"landing_base conf={detection['confidence']:.2f} "
            f"area={detection['area_ratio']:.3f} ex={error_x:.2f}"
        )
        color_label = (
            f"yellow={detection['yellow_ratio_in_bbox']:.2f} "
            f"blue={detection['blue_ratio_in_bbox']:.2f}"
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
        frame_area = max(1.0, float(frame_w * frame_h))
        top_candidates = sorted(
            candidates,
            key=lambda candidate: candidate['area_ratio'],
            reverse=True,
        )[:5]

        return {
            'landing_base_count': len(detections),
            'contour_count': len(contours),
            'candidate_count': len(candidates),
            'valid_candidate_count': len(valid_candidates),
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
                'morph_kernel_size': self.morph_kernel_size,
                'yellow_lower_hsv': list(self.yellow_lower),
                'yellow_upper_hsv': list(self.yellow_upper),
                'blue_lower_hsv': list(self.blue_lower),
                'blue_upper_hsv': list(self.blue_upper),
            },
        }

    @staticmethod
    def _candidate_summary(candidate: Optional[dict]) -> Optional[dict]:
        if candidate is None:
            return None

        return {
            'bbox_xyxy': candidate['bbox_xyxy'],
            'area_ratio': candidate['area_ratio'],
            'yellow_ratio_in_bbox': candidate['yellow_ratio_in_bbox'],
            'blue_ratio_in_bbox': candidate['blue_ratio_in_bbox'],
            'confidence': candidate['confidence'],
            'valid': candidate['valid'],
        }

    def _publish_detections(self, detections: list[dict]) -> None:
        msg = String()
        msg.data = json.dumps(detections, ensure_ascii=False)
        self.detection_pub.publish(msg)

    def _publish_debug(self, debug: dict, detections: list[dict]) -> None:
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
        try:
            out_img = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            out_img.header = header
            self.image_pub.publish(out_img)
        except Exception as exc:
            self.get_logger().error(
                f'Erro ao publicar imagem anotada da base: {exc}',
            )

    def _publish_mask(self, mask, header) -> None:
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
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_detector_warning_time_ns < int(2.0 * 1e9):
            return

        self.get_logger().warn(message)
        self.last_detector_warning_time_ns = now_ns

    def destroy_node(self):
        if self.show_preview:
            try:
                cv2.destroyWindow('Landing Base Preview')
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
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

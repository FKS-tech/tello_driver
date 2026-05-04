#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2

from ultralytics import YOLO
import json

from tello_driver.visual_math import compute_bbox_center, compute_normalized_error


class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')

        self.declare_parameter('input_topic', '/tello/image_raw')
        self.declare_parameter('output_image_topic', '/vision/image_annotated')
        self.declare_parameter('output_detection_topic', '/vision/detections')
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('process_every_n_frames', 3)
        self.declare_parameter('show_preview', True)
        
        self.show_preview = bool(self.get_parameter('show_preview').value)
        self.input_topic = self.get_parameter('input_topic').value
        self.output_image_topic = self.get_parameter('output_image_topic').value
        self.output_detection_topic = self.get_parameter('output_detection_topic').value
        self.model_path = self.get_parameter('model_path').value
        self.confidence_threshold = float(self.get_parameter('confidence_threshold').value)
        self.process_every_n_frames = int(self.get_parameter('process_every_n_frames').value)

        self.bridge = CvBridge()
        self.model = YOLO(self.model_path)

        self.frame_count = 0

        self.image_sub = self.create_subscription(Image, self.input_topic, self.image_callback, 10)
        self.image_pub = self.create_publisher(Image, self.output_image_topic, 10)
        self.detection_pub = self.create_publisher(String, self.output_detection_topic, 10)

        self.get_logger().info(f'VisionNode iniciado')
        self.get_logger().info(f'Entrada: {self.input_topic}')
        self.get_logger().info(f'Saida imagem: {self.output_image_topic}')
        self.get_logger().info(f'Saida deteccoes: {self.output_detection_topic}')
        self.get_logger().info(f'Modelo: {self.model_path}')

    def image_callback(self, msg):
        self.frame_count += 1

        if self.process_every_n_frames > 1:
            if self.frame_count % self.process_every_n_frames != 0:
                return
            
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().error(f'Erro ao converter imagem: {exc}')
            return
        
        try:
            results = self.model(frame, verbose=False)
        except Exception as exc:
            self.get_logger().error(f'Erro na inferencia YOLO: {exc}')
            return
        
        result = results[0]
        annotated = result.plot()

        detections = []
        frame_h, frame_w = frame.shape[:2]

        if result.boxes is not None:
            for box in result.boxes:
                conf = float(box.conf[0])
                if conf < self.confidence_threshold:
                    continue

                cls_id = int(box.cls[0])
                cls_name = self.model.names[cls_id]

                x1, y1, x2, y2 = map(float, box.xyxy[0])
                w = max(0.0, x2 - x1)
                h = max(0.0, y2 - y1)
                area_ratio = (w * h) / float(frame_w * frame_h)
                cx, cy = compute_bbox_center(x1, y1, x2, y2)
                error_x, error_y = compute_normalized_error(cx, cy, frame_w, frame_h)

                detections.append({
                    'class_id': cls_id,
                    'class_name': cls_name,
                    'confidence': conf,
                    'bbox_xyxy': [x1, y1, x2, y2],
                    'area_ratio': area_ratio,
                    'center_px': [cx, cy],
                    'error_norm': [error_x, error_y],
                    'frame_size': [frame_w, frame_h]
                })

        if self.show_preview:
            cv2.imshow("Vision Preview", annotated)
            cv2.waitKey(1)

        try:
            out_img = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            out_img.header = msg.header
            self.image_pub.publish(out_img)
        except Exception as exc:
            self.get_logger().error(f'Erro ao publicar imagem anotada: {exc}')

        try:
            det_msg = String()
            det_msg.data = json.dumps(detections, ensure_ascii=False)
            self.detection_pub.publish(det_msg)
        except Exception as exc:
            self.get_logger().error(f'Erro ao publicar deteccoes: {exc}')


    def destroy_node(self):
        if self.show_preview:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

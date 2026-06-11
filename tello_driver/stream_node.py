#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from tello_driver.tello_client import TelloClient


class TelloVideoNode(Node):
    """Abre o stream de video do Tello e publica frames como sensor_msgs/Image."""

    def __init__(self):
        """Declara parametros, inicializa stream SDK e configura o timer de frames."""
        super().__init__('tello_video_node')

        # Parâmetros
        self.declare_parameter('stream_url', 'udp://0.0.0.0:11111?fifo_size=50000000&overrun_nonfatal=1')
        self.declare_parameter('publish_topic', '/tello/image_raw')
        self.declare_parameter('show_preview', True)
        self.declare_parameter('timer_period', 0.03)
        self.declare_parameter('enable_sdk_init', True)
        self.declare_parameter('enable_stream_on', True)
        self.declare_parameter('command_local_port', 9001)
        self.declare_parameter('sdk_timeout', 5.0)

        self.stream_url = self.get_parameter('stream_url').get_parameter_value().string_value
        self.publish_topic = self.get_parameter('publish_topic').get_parameter_value().string_value
        self.show_preview = self.get_parameter('show_preview').get_parameter_value().bool_value
        self.timer_period = self.get_parameter('timer_period').get_parameter_value().double_value
        self.enable_sdk_init = self.get_parameter('enable_sdk_init').get_parameter_value().bool_value
        self.enable_stream_on = self.get_parameter('enable_stream_on').get_parameter_value().bool_value
        self.command_local_port = self.get_parameter('command_local_port').get_parameter_value().integer_value
        self.sdk_timeout = self.get_parameter('sdk_timeout').get_parameter_value().double_value

        self.bridge = CvBridge()

        # Publicador de imagem
        self.image_pub = self.create_publisher(Image, self.publish_topic, 10)

        self.initialize_tello_stream()

        # Abrir stream
        self.cap = None
        self.open_stream()

        # Timer
        self.timer = self.create_timer(self.timer_period, self.update_frame)

        self.get_logger().info(f"Nó de vídeo iniciado")
        self.get_logger().info(f"Stream: {self.stream_url}")
        self.get_logger().info(f"Publicando em: {self.publish_topic}")
        self.get_logger().info(f"Preview local: {'ON' if self.show_preview else 'OFF'}")

    def initialize_tello_stream(self):
        """Envia `command` e/ou `streamon` antes de abrir o video UDP."""
        if not self.enable_sdk_init and not self.enable_stream_on:
            return

        try:
            tello = TelloClient(
                local_port=self.command_local_port,
                timeout=self.sdk_timeout,
            )
        except Exception as exc:
            self.get_logger().error(f"Erro ao criar cliente SDK do Tello: {exc}")
            return

        try:
            if self.enable_sdk_init:
                response = tello.enter_sdk_mode()
                self.get_logger().info(f"SDK init: {response}")

            if self.enable_stream_on:
                response = tello.stream_on()
                self.get_logger().info(f"Stream on: {response}")
        finally:
            tello.close()

    def open_stream(self):
        """Abre ou reabre o VideoCapture usando a URL configurada."""
        if self.cap is not None:
            self.cap.release()

        self.cap = cv2.VideoCapture(self.stream_url, cv2.CAP_FFMPEG)

        if not self.cap.isOpened():
            self.get_logger().error("Não foi possível abrir o stream de vídeo.")
        else:
            self.get_logger().info("Stream de vídeo aberto com sucesso.")

    def update_frame(self):
        """Captura um frame, publica no ROS e mostra preview se habilitado."""
        if self.cap is None or not self.cap.isOpened():
            self.get_logger().warn("Stream não está aberto. Tentando reconectar...")
            self.open_stream()
            return

        ret, frame = self.cap.read()

        if not ret or frame is None:
            self.get_logger().warn("Sem frame. Tentando reconectar...")
            self.open_stream()
            return

        # Publica frame no tópico ROS
        try:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'tello_camera'
            self.image_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Erro ao converter/publicar frame: {e}")
            return

        # Preview local opcional
        if self.show_preview:
            cv2.imshow("Tello Camera", frame)
            cv2.waitKey(1)

    def destroy_node(self):
        """Libera o VideoCapture e fecha janelas OpenCV no shutdown."""
        if self.cap is not None:
            self.cap.release()

        if self.show_preview:
            cv2.destroyAllWindows()

        super().destroy_node()


def main(args=None):
    """Start stream_node and publish frames from the Tello video stream."""
    rclpy.init(args=args)

    node = TelloVideoNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

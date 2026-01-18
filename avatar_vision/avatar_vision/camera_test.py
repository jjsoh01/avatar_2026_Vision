import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage

import cv2
import numpy as np


class WebcamPublisher(Node):

    def __init__(self):
        super().__init__('webcam_publisher')

        self.publisher = self.create_publisher(
            CompressedImage,
            '/realsense/color/image_raw/compressed',
            10
        )

        # 웹캠 열기 (보통 내장 카메라 = 0)
        self.cap = cv2.VideoCapture(0)

        # 해상도 / FPS 설정
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)

        if not self.cap.isOpened():
            self.get_logger().error('Webcam not opened')
            return

        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)
        self.get_logger().info('Webcam publisher started')

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        # JPEG 압축
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 70]
        success, encoded = cv2.imencode('.jpg', frame, encode_param)
        if not success:
            return

        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'webcam_frame'
        msg.format = 'jpeg'
        msg.data = encoded.tobytes()

        self.publisher.publish(msg)

        # 로컬 확인용
        cv2.imshow('Webcam View', frame)
        cv2.waitKey(1)

    def destroy_node(self):
        if self.cap.isOpened():
            self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WebcamPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

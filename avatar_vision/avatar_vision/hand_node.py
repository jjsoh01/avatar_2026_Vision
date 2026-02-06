import rclpy
from rclpy.node import Node

import cv2
import mediapipe as mp
import numpy as np

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import Header
from cv_bridge import CvBridge


class HandTrackingNode(Node):
    def __init__(self):
        super().__init__('hand_tracking_node')

        self.bridge = CvBridge()

        # 퍼블리셔
        self.joint_pub = self.create_publisher(Point, '/hand/index_tip', 10)
        self.image_pub = self.create_publisher(Image, '/hand/debug_image', 10)

        # MediaPipe 초기화
        self.mp_hands = mp.solutions.hands
        self.mp_draw = mp.solutions.drawing_utils

        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.6,
            min_tracking_confidence=0.6
        )

        # 카메라 (웹캠 / RealSense RGB도 가능)
        self.cap = cv2.VideoCapture(0)

        self.timer = self.create_timer(0.03, self.timer_cb)  # ~30 FPS
        self.get_logger().info("Hand tracking ROS2 node started")

    def timer_cb(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(img_rgb)

        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                self.mp_draw.draw_landmarks(
                    frame,
                    hand_landmarks,
                    self.mp_hands.HAND_CONNECTIONS
                )

                # 검지 끝 (index finger tip = 8)
                h, w, _ = frame.shape
                lm = hand_landmarks.landmark[8]
                cx = lm.x * w
                cy = lm.y * h

                cv2.circle(frame, (int(cx), int(cy)), 6, (0, 0, 255), -1)

                # ROS 메시지 publish
                p = Point()
                p.x = float(cx)
                p.y = float(cy)
                p.z = 0.0
                self.joint_pub.publish(p)

        # 디버그 이미지 publish
        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.image_pub.publish(img_msg)

        # 로컬 창 (선택)
        cv2.imshow("Hand Tracking", frame)
        cv2.waitKey(1)

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HandTrackingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
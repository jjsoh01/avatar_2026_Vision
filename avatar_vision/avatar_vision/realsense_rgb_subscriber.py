import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32, Bool

import cv2
import numpy as np


class RGBSubscriber(Node):
    def __init__(self):
        super().__init__('rgb_subscriber')

        self.create_subscription(
            CompressedImage,
            '/camera/color/image_raw/compressed',
            self.image_cb,
            10
        )
        self.create_subscription(Float32, '/face/distance_m', self.dist_cb, 10)
        self.create_subscription(Bool, '/face/is_approaching', self.approach_cb, 10)
        self.create_subscription(Bool, '/face/is_looking', self.gaze_cb, 10)

        self.distance = None
        self.approaching = False
        self.looking = False

        self.get_logger().info("RGB subscriber started")

    def dist_cb(self, msg):
        self.distance = msg.data

    def approach_cb(self, msg):
        self.approaching = msg.data

    def gaze_cb(self, msg):
        self.looking = msg.data

    def image_cb(self, msg):
        frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            return

        y = 30
        if self.distance is not None:
            cv2.putText(frame, f"Distance: {self.distance:.2f} m", (20, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)
            y += 28

        cv2.putText(frame, f"Approaching: {self.approaching}", (20, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65,
                    (0, 165, 255) if self.approaching else (120, 120, 120), 2)
        y += 28

        cv2.putText(frame, f"Gaze: {self.looking}", (20, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65,
                    (0, 255, 255) if self.looking else (120, 120, 120), 2)

        cv2.imshow("RGB Viewer", frame)
        if cv2.waitKey(1) & 0xFF in [27, ord('q')]:
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = RGBSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

# OpenCV 관련
import cv2
import numpy as np

# Subscriber 노드 생성
class RealSenseRGBSubscriber(Node):
    def __init__(self):
        super().__init__('realsense_rgb_subscriber') # 노드 이름

        self.subscription = self.create_subscription(
            CompressedImage,
            '/realsense/color/image_raw/compressed',   # Publisher와 동일
            self.callback,
            10
        )

        self.get_logger().info(
            'Subscribed to /realsense/color/image_raw/compressed'
        )

# 구독시 할 일
    def callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.unit8)

            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            if frame is not None:
                cv2.imshow('RealSense Compressed RGB (subscriber)', frame)
                cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f'Could not decode image: {e}')
      


def main(args=None):
    rclpy.init(args=args)
    node = RealSenseRGBSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()
  

if __name__ == '__main__':
    main()

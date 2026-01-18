import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage     # Publisher와 동일  **(수정)
import numpy as np      #  **(수정)

# OpenCV 관련
from cv_bridge import CvBridge
import cv2


# Subscriber 노드 생성
class RealSenseRGBSubscriber(Node):
    def __init__(self):
        super().__init__('realsense_rgb_subscriber') # 노드 이름

        self.bridge = CvBridge() # 이번엔 ROS Image -> OpenCV 로 바꾸려고

        self.subscription = self.create_subscription(
            CompressedImage,    # Publisher와 동일  **(수정)
            '/realsense/color/image_raw/compressed',   # Publisher와 동일  **(수정)
            self.callback,
            10
        )

        self.get_logger().info(
            'Subscribed to /realsense/color/image_raw/compressed'
        )  # log를 보기 위해

# 구독시 할 일
    def callback(self, msg):
        
        np_arr = np.frombuffer(msg.data, dtype=np.uint8)    #  **(수정)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)


        # 영상 화면 띄우기  **(수정)   
        cv2.imshow("Compressed Image", frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = RealSenseRGBSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()

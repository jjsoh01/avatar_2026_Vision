import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
# 압축해서 보내기 위해
from sensor_msgs.msg import CompressedImage

# OpenCV 관련
from cv_bridge import CvBridge
import cv2
import numpy as np

# Subscriber 노드 생성
class RealSenseRGBSubscriber(Node):
    def __init__(self):
        super().__init__('realsense_rgb_subscriber') # 노드 이름

        self.bridge = CvBridge() # 이번엔 ROS Image -> OpenCV 로 바꾸려고

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
            # 압축된 byte 데이터를 numpy 1차원 배열로 변환
            np_arr = np.frombuffer(msg.data, np.uint8)

            # OpenCV를 이용해 JPEG 압축 해제
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            # ROS Image → OpenCV 변환
            #frame = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # 영상 화면 띄우기
            if frame is not None:
                cv2.imshow('RealSense RGB (subscriber)', frame)
                cv2.waitKey(1)
            else:
                self.get_logger().warn('Decoding failed: Frame is empty')

        except Exception as e:
            self.get_logger().error(f'Error in callback: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    node = RealSenseRGBSubscriber()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        # 종료 전 자원 해제
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()

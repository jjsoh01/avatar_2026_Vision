import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage # 메시지 타입 변경
import cv2
import numpy as np

class RealSenseCompressedSubscriber(Node):
    def __init__(self):
        super().__init__('realsense_rgb_subscriber')

        self.subscription = self.create_subscription(
            CompressedImage, # 타입 변경
            '/realsense/color/image_raw/compressed', # 토픽 이름 일치
            self.callback,
            10
        )
        self.get_logger().info('Subscribed to Compressed Image')

    def callback(self, msg):
        # 1. 압축된 바이트 데이터를 numpy 배열로 변환
        np_arr = np.frombuffer(msg.data, np.uint8)
        
        # 2. OpenCV를 이용해 JPEG 압축 해제 (디코딩)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if frame is not None:
            # 영상 화면 띄우기
            cv2.imshow('Remote View (Compressed)', frame)
            cv2.waitKey(1)
            self.get_logger().info('Received and displayed compressed image')
        else:
            self.get_logger().warning('Failed to decode image')

def main(args=None):
    rclpy.init(args=args)
    node = RealSenseCompressedSubscriber()
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

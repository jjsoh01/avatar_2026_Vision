import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class BrailleListener(Node):
    def __init__(self):
        super().__init__('braille_listener')
        
        self.bridge = CvBridge()

        # 1. Subscribe to Text
        self.text_sub = self.create_subscription(
            String,
            '/braille_text',
            self.text_callback,
            10)

        # 2. Subscribe to Debug Image
        self.image_sub = self.create_subscription(
            Image,
            '/braille_debug_img',
            self.image_callback,
            10)
            
        self.get_logger().info('Waiting for Braille Data...')

    def text_callback(self, msg):
        # Simply print the text to the console
        print(f"Received Translation: {msg.data}")

    def image_callback(self, msg):
        try:
            # Convert ROS Image -> OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Show on screen
            cv2.imshow("Subscriber View", cv_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f'Could not convert image: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = BrailleListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

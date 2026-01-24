import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class HandTrackingSubscriber(Node):
    def __init__(self):
        super().__init__('hand_sub_node')
        
        # Publisher가 사용하는 토픽 이름 '/hand/gesture'와 타입을 일치시킵니다.
        self.subscription = self.create_subscription(
            String,
            '/hand/gesture',
            self.listener_callback,
            10
        )
        self.subscription  # 변수 사용 경고 방지
        self.get_logger().info("가위바위보 서브스크라이버 노드가 시작되었습니다!")

    def listener_callback(self, msg):
        # 수신된 메시지에 따라 다른 처리를 할 수 있습니다.
        gesture = msg.data
        if gesture == "ROCK":
            self.get_logger().info(f'받은 제스처: ✊ 바위 ({gesture})')
        elif gesture == "SCISSORS":
            self.get_logger().info(f'받은 제스처: ✌️ 가위 ({gesture})')
        elif gesture == "PAPER":
            self.get_logger().info(f'받은 제스처: 🖐️ 보 ({gesture})')
        else:
            self.get_logger().info(f'상태: {gesture}')

def main(args=None):
    rclpy.init(args=args)
    node = HandTrackingSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
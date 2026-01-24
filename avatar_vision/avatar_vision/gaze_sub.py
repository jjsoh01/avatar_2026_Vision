import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point # 발행자와 동일한 메시지 타입 사용

class GazeSubscriber(Node):
    def __init__(self):
        super().__init__('gaze_subscriber')
        
        # 구독자 생성: (메시지 타입, 토픽 이름, 큐 사이즈)
        self.subscription = self.create_subscription(
            Point,
            'gaze_point',
            self.listener_callback,
            10)
        self.subscription # 변수 유지

    def listener_callback(self, msg):
        # 메시지를 받았을 때 실행되는 함수
        # x, y 값은 -0.5 ~ 0.5 사이의 값입니다.
        x_pos = msg.x
        y_pos = msg.y
        
        # 나중에 여기에 실제 모터 제어 로직이 들어갑니다.
        # 지금은 터미널에 출력해서 확인해봅시다.
        self.get_logger().info(f'시선 데이터 수신 -> X: {x_pos:.3f}, Y: {y_pos:.3f}')

def main(args=None):
    rclpy.init(args=args)
    node = GazeSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
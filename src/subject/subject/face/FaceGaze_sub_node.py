import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class FaceGazeSubNode(Node):
    def __init__(self):
        super().__init__('face_gaze_sub_node')
        
        # 1. 구독자 설정: '/face/status' 토픽 수신
        self.subscription = self.create_subscription(
            String,
            '/face/status',
            self.listener_callback,
            10)
        
        self.get_logger().info("얼굴 데이터 구독 노드가 시작되었습니다. 데이터를 기다리는 중...")

    def listener_callback(self, msg):
        # 2. 데이터 파싱 (문자열 분해)
        # 받은 데이터 형식: "Gaze:FRONT|Prox:50%|Status:NORMAL|Emo:SMILE"
        try:
            data_parts = msg.data.split('|')
            status_dict = {}
            for part in data_parts:
                key, value = part.split(':')
                status_dict[key] = value

            # 개별 변수 추출
            gaze = status_dict.get('Gaze', 'FRONT')
            prox = status_dict.get('Prox', '0%')
            dist_status = status_dict.get('Status', 'NORMAL')
            emotion = status_dict.get('Emo', 'NEUTRAL')

            # 3. 데이터에 따른 아바타 리액션 로직 제어
            self.control_avatar(gaze, prox, dist_status, emotion)
            
        except Exception as e:
            self.get_logger().error(f'데이터 파싱 중 오류 발생: {e}')

    def control_avatar(self, gaze, prox, dist_status, emotion):
        """
        여기에서 아바타의 모터나 LED 등을 제어하는 로직을 작성합니다.
        지금은 터미널 로그로 출력을 대체합니다.
        """
        log_msg = f"[수신] 시선: {gaze:5} | 근접도: {prox:4} | 상태: {dist_status:10} | 감정: {emotion}"
        self.get_logger().info(log_msg)

        # 예시 리액션 로직
        if emotion == "SMILE":
            # 아바타 입 주변 LED를 초록색으로 변경하거나 웃는 소리 출력 등
            pass
        
        if dist_status == "TOO CLOSE":
            # 아바타가 뒤로 물러나는 동작 수행 등
            pass

def main(args=None):
    rclpy.init(args=args)
    node = FaceGazeSubNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
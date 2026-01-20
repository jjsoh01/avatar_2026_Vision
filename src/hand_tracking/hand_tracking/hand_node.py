import sys
import os

# 1. 라이브러리 경로 최우선 순위 부여
user_path = os.path.expanduser('~/.local/lib/python3.10/site-packages')
if user_path not in sys.path:
    sys.path.insert(0, user_path)

import rclpy
from rclpy.node import Node
import cv2

# 2. 물리적으로 확인된 '진짜' 경로로 임포트
from mediapipe.python.solutions import hands as mp_hands
from mediapipe.python.solutions import drawing_utils as mp_drawing

from geometry_msgs.msg import Point
from cv_bridge import CvBridge

class HandTrackingNode(Node):
    def __init__(self):
        super().__init__('hand_node')
        self.bridge = CvBridge()
        self.joint_pub = self.create_publisher(Point, '/hand/index_tip', 10)
        
        # MediaPipe 설정
        self.hands = mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.7, 
            min_tracking_confidence=0.7
        )
        self.mp_drawing_utils = mp_drawing
        self.mp_hands_conn = mp_hands.HAND_CONNECTIONS
        
        self.cap = cv2.VideoCapture(0)
        self.create_timer(0.033, self.timer_cb)
        self.get_logger().info("가위바위보 노드가 시작되었습니다!")

    def timer_cb(self):
        ret, frame = self.cap.read()
        if not ret: 
            return

        # 좌우 반전 (거울 효과 - 가위바위보 할 때 더 편합니다)
        frame = cv2.flip(frame, 1)

        img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(img_rgb)

        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                # 손가락 펴짐 여부 확인 리스트
                fingers = []
                
                # 1. 엄지 판별 (거울 반전 기준: 4번이 3번보다 왼쪽에 있으면 편 것)
                if hand_landmarks.landmark[4].x < hand_landmarks.landmark[3].x:
                    fingers.append(1)
                else:
                    fingers.append(0)

                # 2. 나머지 네 손가락 (검지~새끼)
                # Tip(8,12,16,20)이 Pip(6,10,14,18)보다 위에 있으면 편 것
                tip_ids = [8, 12, 16, 20]
                pip_ids = [6, 10, 14, 18]
                
                for t_id, p_id in zip(tip_ids, pip_ids):
                    if hand_landmarks.landmark[t_id].y < hand_landmarks.landmark[p_id].y:
                        fingers.append(1)
                    else:
                        fingers.append(0)

                # 3. 가위바위보 판별 로직
                gesture = "Waiting..."
                if fingers == [0, 0, 0, 0, 0]:
                    gesture = "ROCK"
                elif fingers[1:3] == [1, 1] and fingers[3:5] == [0, 0]:
                    gesture = "SCISSORS"
                elif fingers == [1, 1, 1, 1, 1]:
                    gesture = "PAPER"

                # 화면 표시
                cv2.putText(frame, f'Hand: {gesture}', (30, 100), 
                            cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 3)
                
                self.mp_drawing_utils.draw_landmarks(frame, hand_landmarks, self.mp_hands_conn)

        cv2.imshow("Rock Paper Scissors ROS2", frame)
        cv2.waitKey(1)

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = HandTrackingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
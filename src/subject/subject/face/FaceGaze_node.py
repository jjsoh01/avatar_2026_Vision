import sys
import os

# 1. 라이브러리 경로 최우선 순위 부여
user_path = os.path.expanduser('~/.local/lib/python3.10/site-packages')
if user_path not in sys.path:
    sys.path.insert(0, user_path)

import rclpy
from rclpy.node import Node
import cv2
import numpy as np

# 2. MediaPipe 임포트 전략
try:
    import mediapipe.python.solutions.face_mesh as mp_face_mesh
except Exception:
    from mediapipe.python.solutions import face_mesh as mp_face_mesh

from std_msgs.msg import String

class FaceGazeNode(Node):
    def __init__(self):
        super().__init__('face_gaze_node')
        
        # 통합 상태 정보 발행
        self.status_pub = self.create_publisher(String, '/face/status', 10)
        
        # Face Mesh 초기화
        self.face_mesh = mp_face_mesh.FaceMesh(
            max_num_faces=1,
            refine_landmarks=True,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )
        
        self.cap = cv2.VideoCapture(0)
        self.create_timer(0.05, self.timer_callback)
        self.get_logger().info("정밀 감정 인식 기능이 포함된 노드가 시작되었습니다.")

    def timer_callback(self):
        success, frame = self.cap.read()
        if not success:
            return

        frame = cv2.flip(frame, 1)
        ih, iw, _ = frame.shape
        img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.face_mesh.process(img_rgb)

        # 기본 상태값
        gaze = "FRONT"
        dist_status = "NORMAL"
        emotion = "NEUTRAL"
        proximity = 0
        color = (0, 255, 0) # 기본 초록색

        if results.multi_face_landmarks:
            for face_landmarks in results.multi_face_landmarks:
                lm = face_landmarks.landmark
                
                # [1] 방향 판정 로직
                h_ratio = abs(lm[1].x - lm[234].x) / (abs(lm[1].x - lm[234].x) + abs(lm[1].x - lm[454].x))
                v_ratio = abs(lm[1].y - lm[10].y) / (abs(lm[1].y - lm[10].y) + abs(lm[1].y - lm[152].y))

                if h_ratio < 0.35: gaze = "LEFT"
                elif h_ratio > 0.65: gaze = "RIGHT"
                elif v_ratio < 0.35: gaze = "UP"
                elif v_ratio > 0.58: gaze = "DOWN"

                # [2] 거리 측정 및 근접도(Proximity %) 로직
                p1 = np.array([lm[133].x * iw, lm[133].y * ih])
                p2 = np.array([lm[362].x * iw, lm[362].y * ih])
                eye_dist = np.linalg.norm(p1 - p2)
                
                proximity = (eye_dist - 45) / (95 - 45) * 100
                proximity = max(0, min(100, int(proximity)))

                if proximity > 80:
                    dist_status = "TOO CLOSE"
                    color = (0, 0, 255)
                elif proximity < 20:
                    dist_status = "TOO FAR"
                    color = (255, 0, 0)
                else:
                    dist_status = "NORMAL"
                    color = (0, 255, 0)

                # [3] 개선된 감정 판단 로직
                # 입 너비 정규화 (얼굴 폭 대비 입 폭)
                face_width = abs(lm[234].x - lm[454].x)
                mouth_width = abs(lm[61].x - lm[291].x)
                mouth_width_ratio = mouth_width / face_width

                # 입꼬리 상승도 (중앙선 대비 입꼬리 y값 차이 합)
                mouth_center_y = (lm[0].y + lm[17].y) / 2
                smile_lift = (mouth_center_y - lm[61].y) + (mouth_center_y - lm[291].y)
                
                # 입 벌림 정도
                mouth_open = abs(lm[13].y - lm[14].y)
                
                # 판정 조건
                if mouth_width_ratio > 0.42 and smile_lift > 0.01:
                    emotion = "SMILE"
                elif mouth_open > 0.03:
                    emotion = "SURPRISE"
                else:
                    emotion = "NEUTRAL"

                # 시각화 피드백
                cv2.putText(frame, f"GAZE: {gaze}", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                cv2.putText(frame, f"PROXIMITY: {proximity}% ({dist_status})", (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
                cv2.putText(frame, f"EMO: {emotion}", (20, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

        # ROS 2 메시지 발행
        msg = String()
        msg.data = f"Gaze:{gaze}|Prox:{proximity}%|Status:{dist_status}|Emo:{emotion}"
        self.status_pub.publish(msg)

        cv2.imshow('Precision Face Tracking', frame)
        cv2.waitKey(1)

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = FaceGazeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
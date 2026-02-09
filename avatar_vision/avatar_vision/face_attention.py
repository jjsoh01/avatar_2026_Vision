import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import mediapipe as mp
import numpy as np
from sensor_msgs.msg import CompressedImage

class FaceAttentionNode(Node):
    def __init__(self):
        super().__init__('face_attention_node')

        self.sub = self.create_subscription(
            CompressedImage,
            '/realsense/color/image_raw/compressed',   # RealSense 기준
            self.image_cb,
            10
        )

        self.bridge = CvBridge()
        self.face_mesh = mp.solutions.face_mesh.FaceMesh(
            static_image_mode=False,
            max_num_faces=1,
            refine_landmarks=True,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )

    def image_cb(self, msg):
        img = self.bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')
        h, w, _ = img.shape

        rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        res = self.face_mesh.process(rgb)

        attention_score = 0.0
        yaw_score = 0.0
        pitch_score = 0.0

        if res.multi_face_landmarks:
            lm = res.multi_face_landmarks[0].landmark

            # 주요 landmark
            left_eye = lm[33]
            right_eye = lm[263]
            nose = lm[1]

            # ------------------
            # YAW 계산 (좌우 회전)
            # ------------------
            face_center_x = (left_eye.x + right_eye.x) / 2.0
            yaw_offset = abs(face_center_x - 0.5)  # 화면 중심 기준
            yaw_score = max(0.0, 1.0 - yaw_offset * 2.0)

            # ------------------
            # PITCH 계산 (상하 회전)
            # ------------------
            eye_center_y = (left_eye.y + right_eye.y) / 2.0
            pitch_offset = abs(nose.y - eye_center_y)
            pitch_score = max(0.0, 1.0 - pitch_offset * 3.0)

            # ------------------
            # 최종 attention score
            # ------------------
            attention_score = yaw_score * pitch_score

            # ------------------
            # 시각화
            # ------------------
            lx, ly = int(left_eye.x * w), int(left_eye.y * h)
            rx, ry = int(right_eye.x * w), int(right_eye.y * h)
            nx, ny = int(nose.x * w), int(nose.y * h)

            cv2.circle(img, (lx, ly), 3, (0, 255, 0), -1)
            cv2.circle(img, (rx, ry), 3, (0, 255, 0), -1)
            cv2.circle(img, (nx, ny), 4, (0, 0, 255), -1)

            # 얼굴 중심선
            cx = int(face_center_x * w)
            cv2.line(img, (cx, 0), (cx, h), (255, 0, 0), 2)

        # 화면 중앙 기준선
        cv2.line(img, (w // 2, 0), (w // 2, h), (0, 0, 255), 1)

        # 텍스트 출력
        cv2.putText(img, f'Yaw Score   : {yaw_score:.2f}', (30, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
        cv2.putText(img, f'Pitch Score : {pitch_score:.2f}', (30, 75),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
        cv2.putText(img, f'Attention   : {attention_score:.2f}', (30, 110),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2)

        # 상태 표시
        state = "LOOKING"
        if attention_score < 0.4:
            state = "NOT LOOKING"

        cv2.putText(img, f'STATE: {state}', (30, 150),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                    (0, 255, 0) if state == "LOOKING" else (0, 0, 255), 2)

        cv2.imshow('Face Attention (Yaw + Pitch)', img)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = FaceAttentionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
import rclpy
from rclpy.node import Node
import cv2
import mediapipe as mp
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool, Float32, String
from cv_bridge import CvBridge

class SmartFaceNode(Node):
    def __init__(self):
        super().__init__('smart_face_node')
        self.bridge = CvBridge()

        # 퍼블리셔 설정
        self.pose_pub = self.create_publisher(Vector3, '/face/head_pose', 10)
        self.gaze_pub = self.create_publisher(Bool, '/face/is_gazing', 10)
        self.dist_pub = self.create_publisher(Float32, '/face/distance', 10)
        self.emotion_pub = self.create_publisher(String, '/face/emotion', 10)
        self.image_pub = self.create_publisher(Image, '/face/debug_image', 10)

        # MediaPipe Face Mesh (눈동자 추적 포함)
        self.mp_face_mesh = mp.solutions.face_mesh
        self.face_mesh = self.mp_face_mesh.FaceMesh(
            max_num_faces=1,
            refine_landmarks=True,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )

        self.cap = cv2.VideoCapture(0)
        self.timer = self.create_timer(0.03, self.timer_cb)

        # 3D 모델 좌표 (PnP용)
        self.model_points = np.array([
            (0.0, 0.0, 0.0),             # 코 끝
            (0.0, -330.0, -65.0),        # 턱
            (-225.0, 170.0, -135.0),     # 왼쪽 눈 끝
            (225.0, 170.0, -135.0),      # 오른쪽 눈 끝
            (-150.0, -150.0, -125.0),    # 왼쪽 입 끝
            (150.0, -150.0, -125.0)      # 오른쪽 입 끝
        ])

    def timer_cb(self):
        ret, frame = self.cap.read()
        if not ret: return

        h, w, _ = frame.shape
        img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.face_mesh.process(img_rgb)

        if results.multi_face_landmarks:
            for face_landmarks in results.multi_face_landmarks:
                # --- 1. Head Pose 계산 (이전과 동일) ---
                image_points = np.array([
                    (face_landmarks.landmark[1].x * w, face_landmarks.landmark[1].y * h),
                    (face_landmarks.landmark[152].x * w, face_landmarks.landmark[152].y * h),
                    (face_landmarks.landmark[33].x * w, face_landmarks.landmark[33].y * h),
                    (face_landmarks.landmark[263].x * w, face_landmarks.landmark[263].y * h),
                    (face_landmarks.landmark[61].x * w, face_landmarks.landmark[61].y * h),
                    (face_landmarks.landmark[291].x * w, face_landmarks.landmark[291].y * h)
                ], dtype="double")

                focal_length = w
                center = (w/2, h/2)
                cam_matrix = np.array([[focal_length, 0, center[0]], [0, focal_length, center[1]], [0, 0, 1]], dtype="double")
                success, rot_vec, trans_vec = cv2.solvePnP(self.model_points, image_points, cam_matrix, np.zeros((4,1)))
                rmat, _ = cv2.Rodrigues(rot_vec)
                _, _, _, _, _, _, angles = cv2.decomposeProjectionMatrix(np.hstack((rmat, trans_vec)))
                pitch, yaw, roll = angles.flatten()
                pitch *= -1

                # --- 2. 눈 기준 응시 판단 (Gaze Tracking) ---
                # 왼쪽 눈의 가로 범위 내 눈동자 위치 비율 계산
                l_iris = face_landmarks.landmark[468].x
                l_inner = face_landmarks.landmark[133].x
                l_outer = face_landmarks.landmark[33].x
                # 눈동자가 중앙에 있을수록 0.5에 가까움
                l_ratio = (l_iris - l_outer) / (l_inner - l_outer) if (l_inner - l_outer) != 0 else 0.5

                # 고개 각도와 눈동자 위치를 결합하여 최종 응시 판단
                is_gazing_val = bool(abs(yaw) < 20 and 0.4 < l_ratio < 0.6)
                
                gaze_msg = Bool()
                gaze_msg.data = is_gazing_val
                self.gaze_pub.publish(gaze_msg)

                # --- 3. 거리 측정 (Distance Estimation) ---
                # 양쪽 눈 사이의 픽셀 거리 측정
                lx, ly = face_landmarks.landmark[33].x * w, face_landmarks.landmark[33].y * h
                rx, ry = face_landmarks.landmark[263].x * w, face_landmarks.landmark[263].y * h
                pixel_dist = np.sqrt((lx-rx)**2 + (ly-ry)**2)
                
                # 가상의 초점거리를 이용한 미터 환산 (실제 카메라에 맞춰 조정 필요)
                # 실제 눈 사이 거리(평균 63mm) / 픽셀 거리 * 초점계수
                dist_m = (63.0 * focal_length) / (pixel_dist * 1000.0) 
                
                dist_msg = Float32()
                dist_msg.data = float(dist_m)
                self.dist_pub.publish(dist_msg)

                # --- 4. 간단한 감정 인식 (Emotion) ---
                # 입꼬리와 코 사이의 거리 등으로 미소 판별
                mouth_w = abs(face_landmarks.landmark[61].x - face_landmarks.landmark[291].x)
                mouth_h = abs(face_landmarks.landmark[13].y - face_landmarks.landmark[14].y)
                
                emotion = "Neutral"
                if mouth_h > 0.05: emotion = "Surprised"
                elif mouth_w > 0.10: emotion = "Happy" # 입이 가로로 커지면 웃음으로 간주
                
                emo_msg = String()
                emo_msg.data = emotion
                self.emotion_pub.publish(emo_msg)

                # --- 시각화 ---
                self.draw_debug(frame, image_points, is_gazing_val, dist_m, emotion, l_ratio)

        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.image_pub.publish(img_msg)
        cv2.imshow("Smart Face Tracking", frame)
        cv2.waitKey(1)

    def draw_debug(self, frame, points, gazing, dist, emotion, iris_ratio):
        color = (0, 255, 0) if gazing else (0, 0, 255)
        # 응시 여부 및 정보 출력
        cv2.putText(frame, f"Gaze: {gazing} (Ratio:{iris_ratio:.2f})", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        cv2.putText(frame, f"Dist: {dist:.2f}m", (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        cv2.putText(frame, f"Emotion: {emotion}", (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 2)
        # 코에 점 표시
        cv2.circle(frame, (int(points[0][0]), int(points[0][1])), 3, (255, 0, 0), -1)

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SmartFaceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
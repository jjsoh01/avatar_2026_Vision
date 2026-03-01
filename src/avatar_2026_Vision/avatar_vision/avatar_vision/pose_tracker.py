import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import cv2
import numpy as np
from ultralytics import YOLO
import time
from collections import deque

class PoseTrackerNode(Node):
    def __init__(self):
        super().__init__('pose_tracker_node')
        # 1. 모델 로드 (선이 안 보인다면 이 모델이 정상 로드되었는지 확인 필수)
        self.model = YOLO('yolov8n-pose.pt')
        
        # 2. 로봇 제어 퍼블리셔
        self.joint_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        
        # 3. 카메라 연결
        self.cap = cv2.VideoCapture(0)
        
        # 조인트 이름 정의 (R1~6, L1~6, N1~2 순서 일치)
        self.joint_names = [
            'right_joint1', 'right_joint2', 'right_joint3', 'right_joint4', 'right_joint5', 'right_joint6',
            'left_joint1', 'left_joint2', 'left_joint3', 'left_joint4', 'left_joint5', 'left_joint6',
            'neck_joint1', 'neck_joint2'
        ]
        
        # --- 설정치 ---
        self.wait_duration = 1.0  # 1초 정지 대기
        self.threshold = 0.12     # 움직임 감지 문턱값
        self.queue_size = 7       # 필터 크기
        
        self.history = deque(maxlen=self.queue_size)
        self.prev_avg_pos = np.zeros(14)
        self.last_move_time = time.time()
        self.is_stable = False

        # 20 FPS 수준으로 실행
        self.timer = self.create_timer(0.05, self.process_pose)
        self.get_logger().info("=== 아바타 전신 추적 및 시각화 노드 가동 ===")

    def get_angle(self, p1, p2, p3):
        """세 점 사이의 사이각 계산"""
        a, b, c = np.array(p1), np.array(p2), np.array(p3)
        ba, bc = a - b, c - b
        cosine_angle = np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc) + 1e-6)
        return np.arccos(np.clip(cosine_angle, -1.0, 1.0))

    def process_pose(self):
        ret, frame = self.cap.read()
        if not ret: return

        # YOLO 추론
        results = self.model(frame, verbose=False)
        current_raw = np.zeros(14)
        
        # [핵심] 선(스켈레톤)이 그려진 프레임 생성
        annotated_frame = results[0].plot()

        for r in results:
            if r.keypoints is not None and len(r.keypoints.xyn) > 0:
                kp = r.keypoints.xyn[0].tolist()
                
                # --- [1. 오른팔: 방향 반전 보정] ---
                if kp[6][0] > 0 and kp[12][0] > 0 and kp[8][0] > 0:
                    r_shld = self.get_angle(kp[12], kp[6], kp[8])
                    current_raw[1] = -float(r_shld) # 어깨 방향 반전
                if kp[6][0] > 0 and kp[8][0] > 0 and kp[10][0] > 0:
                    r_elbow = self.get_angle(kp[6], kp[8], kp[10])
                    current_raw[3] = -float(np.pi - r_elbow) # 팔꿈치 방향 반전

                # --- [2. 왼팔 활성화] ---
                if kp[5][0] > 0 and kp[11][0] > 0 and kp[7][0] > 0:
                    l_shld = self.get_angle(kp[11], kp[5], kp[7])
                    current_raw[7] = float(l_shld)
                if kp[5][0] > 0 and kp[7][0] > 0 and kp[9][0] > 0:
                    l_elbow = self.get_angle(kp[5], kp[7], kp[9])
                    current_raw[9] = float(np.pi - l_elbow)

                # --- [3. 목 활성화] ---
                if kp[0][0] > 0:
                    # 목 좌우 (Index 13)
                    current_raw[13] = float(-(kp[0][0] - 0.5) * 2.0)
                    # 목 상하 (Index 12)
                    shoulder_center_y = (kp[5][1] + kp[6][1]) / 2
                    current_raw[12] = float(-(kp[0][1] - (shoulder_center_y - 0.1)) * 2.5)

        # 노이즈 필터링 및 정지 판정
        self.history.append(current_raw)
        avg_current_pos = np.mean(self.history, axis=0)
        diff = np.max(np.abs(avg_current_pos - self.prev_avg_pos))

        if diff > self.threshold:
            self.last_move_time = time.time()
            self.is_stable = False
            status_txt, color = f"MOVING ({diff:.2f})", (0, 0, 255)
        else:
            elapsed = time.time() - self.last_move_time
            if elapsed >= self.wait_duration:
                if not self.is_stable:
                    self.is_stable = True
                    self.publish_joints(avg_current_pos)
                status_txt, color = "POSE CAPTURED!", (0, 255, 0)
            else:
                status_txt, color = f"HOLD... {elapsed:.1f}s", (0, 255, 255)

        self.prev_avg_pos = avg_current_pos
        
        # 화면에 상태 텍스트 출력
        cv2.putText(annotated_frame, status_txt, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, color, 2)
        
        # 최종 윈도우 출력
        cv2.imshow("Avatar Vision Tracker", annotated_frame)
        cv2.waitKey(1)

    def publish_joints(self, positions):
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = positions.tolist()
        point.time_from_start.sec = 1  # 1초 동안 부드럽게 이동
        msg.points.append(point)
        self.joint_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PoseTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import time
from collections import deque
import pyrealsense2 as rs

class Yolo3DPoseTrackerNode(Node):
    def __init__(self):
        super().__init__('yolo_3d_pose_tracker_node')
        
        # 1. 모델 및 브릿지 초기화
        self.model = YOLO('yolov8n-pose.pt')
        self.bridge = CvBridge()
        
        # 2. RealSense 카메라 파라미터 (내부 파라미터 설정 필수)
        # 실제 카메라의 fx, fy, ppx, ppy 값으로 수정하면 더 정확합니다.
        self.intr = rs.intrinsics()
        self.intr.width, self.intr.height = 640, 480
        self.intr.fx, self.intr.fy = 615.0, 615.0  # 예시 값
        self.intr.ppx, self.intr.ppy = 320.0, 240.0
        self.intr.model = rs.distortion.none
        self.intr.coeffs = [0, 0, 0, 0, 0]
        self.depth_scale = 0.001 # 16-bit depth to meters

        # 3. 로봇 제어 설정
        self.joint_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.joint_names = [
            'right_joint1', 'right_joint2', 'right_joint3', 'right_joint4', 'right_joint5', 'right_joint6',
            'left_joint1', 'left_joint2', 'left_joint3', 'left_joint4', 'left_joint5', 'left_joint6',
            'neck_joint1', 'neck_joint2'
        ]
        
        # 4. 구독 설정 (RealSense 노드로부터 데이터 수신)
        self.create_subscription(CompressedImage, '/realsense/color/image_raw/compressed', self.rgb_cb, 10)
        self.create_subscription(Image, '/realsense/aligned_depth_to_color/image_raw', self.depth_cb, 10)

        # 5. 상태 관리 변수 (기존 로직 유지)
        self.latest_depth = None
        self.history = deque(maxlen=7)
        self.prev_avg_pos = np.zeros(14)
        self.last_move_time = time.time()
        self.is_stable = False
        self.threshold = 0.15 # 3D 공간 미터 단위이므로 문턱값 조정 필요

        self.get_logger().info("=== YOLO 3D 아바타 추적 노드 가동 ===")

    def depth_cb(self, msg):
        self.latest_depth = self.bridge.imgmsg_to_cv2(msg, "16UC1")

    def get_3d_point(self, u, v, depth_img):
        """픽셀 좌표와 Depth를 이용해 3D 공간 좌표(x, y, z) 반환"""
        h, w = depth_img.shape
        u, v = int(max(0, min(w-1, u))), int(max(0, min(h-1, v)))
        
        # 3x3 영역의 중앙값 사용으로 노이즈 제거
        window = depth_img[max(0, v-1):v+2, max(0, u-1):u+2]
        z_raw = np.median(window[window > 0]) if np.any(window > 0) else 0
        
        if z_raw == 0: return None
        
        z_m = z_raw * self.depth_scale
        # 역투영 (Deproject)
        x = (u - self.intr.ppx) * z_m / self.intr.fx
        y = (v - self.intr.ppy) * z_m / self.intr.fy
        return np.array([x, y, z_m])

    def get_3d_angle(self, p1, p2, p3):
        """3D 공간상 세 점 사이의 벡터 사이각 계산"""
        if p1 is None or p2 is None or p3 is None: return 0.0
        v1 = p1 - p2
        v2 = p3 - p2
        norm = np.linalg.norm(v1) * np.linalg.norm(v2)
        if norm == 0: return 0.0
        return np.arccos(np.clip(np.dot(v1, v2) / norm, -1.0, 1.0))

    def rgb_cb(self, msg):
        if self.latest_depth is None: return
        
        # 이미지 디코딩
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        h, w = frame.shape[:2]

        # YOLO 추론
        results = self.model(frame, verbose=False)
        current_raw = np.zeros(14)
        annotated_frame = results[0].plot()

        if len(results[0].keypoints) > 0:
            kp = results[0].keypoints.xy[0].cpu().numpy() # 2D 픽셀 좌표
            
            # 관절 ID 매핑 (YOLO 기준): 5:L_sh, 6:R_sh, 7:L_el, 8:R_el, 9:L_wr, 10:R_wr, 11:L_hip, 12:R_hip
            # 3D 좌표 추출
            pts = {}
            for idx in [0, 5, 6, 7, 8, 9, 10, 11, 12]:
                pts[idx] = self.get_3d_point(kp[idx][0], kp[idx][1], self.latest_depth)

            # 3D 기반 각도 계산
            # --- 오른팔 ---
            if pts[6] is not None and pts[8] is not None:
                # 어깨 각도 (몸통 대비 상박)
                r_shld = self.get_3d_angle(pts[12], pts[6], pts[8]) if pts[12] is not None else 1.5
                current_raw[1] = -float(r_shld - np.pi/2)
                # 팔꿈치 각도
                if pts[10] is not None:
                    r_elbow = self.get_3d_angle(pts[6], pts[8], pts[10])
                    current_raw[3] = -float(np.pi - r_elbow)

            # --- 왼팔 ---
            if pts[5] is not None and pts[7] is not None:
                l_shld = self.get_3d_angle(pts[11], pts[5], pts[7]) if pts[11] is not None else 1.5
                current_raw[7] = float(l_shld - np.pi/2)
                if pts[9] is not None:
                    l_elbow = self.get_3d_angle(pts[5], pts[7], pts[9])
                    current_raw[9] = float(np.pi - l_elbow)

        # --- 기존 정지 판정 및 필터링 로직 ---
        self.history.append(current_raw)
        avg_pos = np.mean(self.history, axis=0)
        diff = np.max(np.abs(avg_pos - self.prev_avg_pos))

        if diff > self.threshold:
            self.last_move_time = time.time()
            self.is_stable = False
            status, color = f"MOVING ({diff:.2f})", (0, 0, 255)
        else:
            if time.time() - self.last_move_time >= 1.0:
                if not self.is_stable:
                    self.is_stable = True
                    self.publish_joints(avg_pos)
                status, color = "3D POSE CAPTURED!", (0, 255, 0)
            else:
                status, color = "HOLD...", (0, 255, 255)

        self.prev_avg_pos = avg_pos
        cv2.putText(annotated_frame, status, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)
        cv2.imshow("YOLO 3D Tracker", annotated_frame)
        cv2.waitKey(1)

    def publish_joints(self, positions):
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = positions.tolist()
        point.time_from_start.nanosec = 500000000 # 0.5s 이동
        msg.points.append(point)
        self.joint_pub.publish(msg)

def main():
    rclpy.init()
    node = Yolo3DPoseTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
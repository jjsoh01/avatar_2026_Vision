import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from cv_bridge import CvBridge
import cv2
import mediapipe as mp
import numpy as np
import math

class PoseEstimationNode(Node):
    def __init__(self):
        super().__init__('pose_estimation_node')

        # ===============================================================
        # 🔧 [튜닝 파라미터] 여기만 수정하면 방향/각도를 쉽게 조절할 수 있습니다!
        # ===============================================================
        
        # 1. 오른쪽 어깨 (Right Shoulder)
        # 설명: 방향이 반대면 gain을 -1.0으로, 각도가 모자라면 offset을 조절하세요.
        self.right_shoulder_gain = -1.0    # 움직임 방향 (1.0: 정방향, -1.0: 반대방향)
        self.right_shoulder_offset = -1.0   # 초기 각도 보정 (라디안)

        # 2. 왼쪽 어깨 (Left Shoulder)
        # 설명: 90도(약 1.57) 정도 오므리고 있어서 바깥으로 벌려줌
        self.left_shoulder_gain = 1.0      
        self.left_shoulder_offset = 1.57   # +90도 보정

        # ===============================================================

        # 로봇 관절 이름 (YAML 설정과 일치해야 함)
        self.joint_names = [
            'right_joint1', 'right_joint2', 'right_joint3', 'right_joint4', 'right_joint5', 'right_joint6',
            'left_joint1',  'left_joint2',  'left_joint3',  'left_joint4',  'left_joint5',  'left_joint6'
        ]

        # 관절 가동 범위 (Safety Boundary)
        self.joint_limits = {
            'right_joint1': {'min': -1.5, 'max': 1.5}, 'right_joint2': {'min': -2.0, 'max': 2.0},
            'right_joint3': {'min': -2.5, 'max': 0.0}, 'right_joint4': {'min': -1.5, 'max': 1.5},
            'right_joint5': {'min': -1.0, 'max': 1.0}, 'right_joint6': {'min': -1.5, 'max': 1.5},
            
            'left_joint1': {'min': -1.5, 'max': 1.5}, 'left_joint2': {'min': -2.0, 'max': 2.0},
            'left_joint3': {'min': -2.5, 'max': 0.0}, 'left_joint4': {'min': -1.5, 'max': 1.5},
            'left_joint5': {'min': -1.0, 'max': 1.0}, 'left_joint6': {'min': -1.5, 'max': 1.5}
        }

        # MediaPipe & CV 설정
        self.mp_pose = mp.solutions.pose
        self.pose = self.mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5)
        self.mp_drawing = mp.solutions.drawing_utils
        self.bridge = CvBridge()

        # ROS 통신 설정
        self.sub_img = self.create_subscription(
            CompressedImage, '/realsense/color/image_raw/compressed', self.image_callback, 10)
        self.pub_traj = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10)

        self.get_logger().info("Dual Arm Pose Node Started with Tuning Parameters.")

    def image_callback(self, msg):
        try:
            frame = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except: return

        img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.pose.process(img_rgb)

        if results.pose_landmarks:
            self.mp_drawing.draw_landmarks(frame, results.pose_landmarks, self.mp_pose.POSE_CONNECTIONS)
            lm = results.pose_world_landmarks.landmark
            
            # 좌표 추출 (오른쪽: 12,14,16 / 왼쪽: 11,13,15)
            r_sh = np.array([lm[12].x, lm[12].y, lm[12].z])
            r_el = np.array([lm[14].x, lm[14].y, lm[14].z])
            r_wr = np.array([lm[16].x, lm[16].y, lm[16].z])
            
            l_sh = np.array([lm[11].x, lm[11].y, lm[11].z])
            l_el = np.array([lm[13].x, lm[13].y, lm[13].z])
            l_wr = np.array([lm[15].x, lm[15].y, lm[15].z])

            hip_center = (np.array([lm[23].x, lm[23].y, lm[23].z]) + np.array([lm[24].x, lm[24].y, lm[24].z])) / 2.0

            # 각도 계산 (튜닝 파라미터 적용)
            r_joints = self.calculate_arm_angles(r_sh, r_el, r_wr, hip_center, is_right=True)
            l_joints = self.calculate_arm_angles(l_sh, l_el, l_wr, hip_center, is_right=False)
            
            self.publish_joints(r_joints + l_joints)

        cv2.imshow("Pose Estimation", frame)
        cv2.waitKey(1)

    def calculate_arm_angles(self, shoulder, elbow, wrist, hip, is_right=True):
        vec_torso = hip - shoulder
        vec_upper = elbow - shoulder
        vec_lower = wrist - elbow

        # 기본 각도 계산 (어깨-몸통 사이 각도)
        base_angle = self.get_angle_between_vectors(vec_torso, vec_upper)
        # 팔꿈치 각도 (상박-하박)
        elbow_angle = self.get_angle_between_vectors(vec_upper, vec_lower) - math.pi

        # ===== [핵심] 튜닝 파라미터 적용 로직 =====
        if is_right:
            # 오른쪽: (계산된 각도 - 90도) * 게인 + 오프셋
            # 게인이 -1.0이면 방향이 반대가 됨
            j2_angle = (base_angle - (math.pi/2)) * self.right_shoulder_gain + self.right_shoulder_offset
        else:
            # 왼쪽: (계산된 각도 - 90도) * 게인 + 오프셋
            j2_angle = (base_angle - (math.pi/2)) * self.left_shoulder_gain + self.left_shoulder_offset

        # 관절 매핑 (로봇 구조에 따라 j1, j2 순서 확인 필요. 여기선 j2가 상하/벌림이라 가정)
        target = [0.0, j2_angle, elbow_angle, 0.0, 0.0, 0.0]

        # Boundary 적용
        prefix = 'right' if is_right else 'left'
        final_joints = []
        for i, val in enumerate(target):
            joint_name = f"{prefix}_joint{i+1}"
            final_joints.append(self.clamp(val, joint_name))

        return final_joints

    def get_angle_between_vectors(self, u, v):
        dot = np.dot(u, v)
        norms = np.linalg.norm(u) * np.linalg.norm(v)
        if norms == 0: return 0.0
        return math.acos(np.clip(dot / norms, -1.0, 1.0))

    def clamp(self, val, name):
        if name not in self.joint_limits: return val
        limit = self.joint_limits[name]
        return max(min(val, limit['max']), limit['min'])

    def publish_joints(self, joints):
        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = joints
        point.time_from_start.sec = 0; point.time_from_start.nanosec = 100000000
        traj.points.append(point)
        self.pub_traj.publish(traj)

def main(args=None):
    rclpy.init(args=args)
    node = PoseEstimationNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()

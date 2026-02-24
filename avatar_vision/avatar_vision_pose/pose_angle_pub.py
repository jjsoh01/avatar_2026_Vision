import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float32MultiArray
import numpy as np
import math

class PoseEstimationNode(Node):
    def __init__(self):
        super().__init__('pose_estimation_node')

        # ===============================================================
        # [튜닝 파라미터] 로봇의 초기 자세와 방향에 맞게 조절하세요.
        # ===============================================================
        self.right_shoulder_gain = -1.0    
        self.right_shoulder_offset = 0.0   

        self.left_shoulder_gain = 1.0      
        self.left_shoulder_offset = 0.0   
        # ===============================================================

        self.joint_names = [
            'right_joint1', 'right_joint2', 'right_joint3', 'right_joint4', 'right_joint5', 'right_joint6',
            'left_joint1',  'left_joint2',  'left_joint3',  'left_joint4',  'left_joint5',  'left_joint6'
        ]

        # 관절 가동 범위 (Safety Boundary) (수정필요)
        self.joint_limits = {
            'right_joint1': {'min': -1.5, 'max': 1.5}, 'right_joint2': {'min': -2.0, 'max': 2.0},
            'right_joint3': {'min': -2.5, 'max': 0.5}, 'right_joint4': {'min': -1.5, 'max': 1.5},
            'right_joint5': {'min': -1.0, 'max': 1.0}, 'right_joint6': {'min': -1.5, 'max': 1.5},
            
            'left_joint1': {'min': -1.5, 'max': 1.5}, 'left_joint2': {'min': -2.0, 'max': 2.0},
            'left_joint3': {'min': -2.5, 'max': 0.5}, 'left_joint4': {'min': -1.5, 'max': 1.5},
            'left_joint5': {'min': -1.0, 'max': 1.0}, 'left_joint6': {'min': -1.5, 'max': 1.5}
        }

        self.sub_pose = self.create_subscription(
            Float32MultiArray, "/mediapipe/pose_points_3d", self.pose_callback, 10)
        self.pub_traj = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10)

        self.get_logger().info("Dual Arm Pose Node Started.")

    def pose_callback(self, msg):
        # 퍼블리셔 데이터 구조: [ID, X, Y, Z, ID, X, Y, Z, ...]
        # 수신 ID 순서: 11, 12, 13, 14, 15, 16 (어깨L, 어깨R, 팔꿈치L, 팔꿈치R, 손목L, 손목R)
        data = msg.data
        if len(data) < 24: # 6개 포인트 * 4개 데이터
            return

        # 데이터 파싱 (인덱스 주의)
        l_sh = np.array([data[1],  data[2],  data[3]])
        r_sh = np.array([data[5],  data[6],  data[7]])
        l_el = np.array([data[9],  data[10], data[11]])
        r_el = np.array([data[13], data[14], data[15]])
        l_wr = np.array([data[17], data[18], data[19]])
        r_wr = np.array([data[21], data[22], data[23]])

        # 각도 계산 (hip 데이터가 없으므로 내부에서 수직 벡터 사용)
        r_joints = self.calculate_arm_angles(r_sh, r_el, r_wr, is_right=True)
        l_joints = self.calculate_arm_angles(l_sh, l_el, l_wr, is_right=False)

        self.publish_joints(r_joints + l_joints)

    def calculate_arm_angles(self, shoulder, elbow, wrist, is_right=True):
        # 몸통 기준 벡터 (위에서 아래로 향하는 수직 벡터 가정)
        vec_torso = np.array([0, 1, 0]) 
        vec_upper = elbow - shoulder
        vec_lower = wrist - elbow

        # 어깨 벌림 각도 (상박과 몸통 사이)
        base_angle = self.get_angle_between_vectors(vec_torso, vec_upper)
        
        # 팔꿈치 굽힘 각도 (상박과 하박 사이)
        # 일직선일 때 0도가 되도록 pi에서 뺌
        elbow_angle = -(math.pi - self.get_angle_between_vectors(vec_upper, vec_lower))

        if is_right:
            j2_angle = (base_angle - (math.pi/2)) * self.right_shoulder_gain + self.right_shoulder_offset
        else:
            j2_angle = (base_angle - (math.pi/2)) * self.left_shoulder_gain + self.left_shoulder_offset

        # [J1, J2, J3, J4, J5, J6] 매핑
        # J2: 어깨 상하/벌림, J3: 팔꿈치 굽힘
        target = [0.0, j2_angle, elbow_angle, 0.0, 0.0, 0.0]

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
        limit = self.joint_limits.get(name, {'min': -3.14, 'max': 3.14})
        return max(min(val, limit['max']), limit['min'])

    def publish_joints(self, joints):
        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = [float(j) for j in joints]
        point.time_from_start.nanosec = 100000000 # 0.1s
        traj.points.append(point)
        self.pub_traj.publish(traj)

def main(args=None):
    rclpy.init(args=args)
    node = PoseEstimationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
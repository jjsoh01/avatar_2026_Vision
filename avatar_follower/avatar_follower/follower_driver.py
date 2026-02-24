from cmath import pi
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from std_msgs.msg import Float32MultiArray


class FollowerPassthroughDriver(Node):
    def __init__(self):
        super().__init__('follower_passthrough_driver')

        # Action Client for FollowJointTrajectory
        self._action_client = ActionClient(
            self, 
            FollowJointTrajectory, 
            '/arm_controller/follow_joint_trajectory'
        )

        self.leader_sub = self.create_subscription(
            JointState,
            '/leader/joint_states',
            self.leader_callback,
            10
        )

        self.vision_sub = self.create_subscription(
            Float32MultiArray,
            '/avatar/target_angles',
            self.vision_callback,
            10
        )
        
        self.get_logger().info('Follower Passthrough Driver Node has been started.')

    def leader_callback(self, msg: JointState):
        # 액션 서버가 준비되었는지 확인
        if not self._action_client.wait_for_server(timeout_sec=0.1):
            self.get_logger().warn('Action server not available yet. Waiting...')
            return

        # JointState -> JointTrajectory 변환
        traj = JointTrajectory()
        traj.joint_names = list(msg.name)

        point = JointTrajectoryPoint()
        for i in range(len(msg.position)):
            point.positions.append(msg.position[i] - pi)  # 180도 오프셋 적용
            if (msg.name[i]=="right_joint_gripper"):
                point.positions[i] = - 2.0 * point.positions[i]  # 그리퍼 각도 반전
            if (msg.name[i]=="left_joint_gripper"):
                point.positions[i] = - 2.0 * point.positions[i]
            
        
        point.time_from_start = Duration(sec=0, nanosec=20_000_000)  # 0.02s (50 Hz)

        traj.points = [point]

        # 액션 목표(Goal) 생성 및 데이터 복사
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = traj

        # 비동기적으로 목표 전송
        self._action_client.send_goal_async(goal_msg)

    def vision_callback(self, msg: Float32MultiArray):   
        # rate = 5.0 # 각도 조절 비율 (값이 클수록 더 작은 각도로 변환)     
        traj = JointTrajectory()
        traj.joint_names = list(["neck_joint1", "neck_joint2"])
        traj.points = [JointTrajectoryPoint()]
        pitch = -msg.data[2]
        yaw = -msg.data[1]
        min_pitch = 0.01 
        min_yaw = 0.05  # 최소 각도 (0.05 라디안 = 약 2.86도)
        if abs(pitch) > min_pitch:
            pitch = min_pitch * (pitch / abs(pitch))  
        if abs(yaw) > min_yaw:
            yaw = min_yaw * (yaw / abs(yaw))  
      
        traj.points[0].positions = [pitch, yaw]
        traj.points[0].time_from_start = Duration(sec=0, nanosec=20_000_000)  # 0.02s (50 Hz)
        
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = traj

        # 비동기적으로 목표 전송
        self._action_client.send_goal_async(goal_msg)
        self.get_logger().info(f"Received vision angles: {msg.data}, sent to action server.")
        

def main(args=None):
    rclpy.init(args=args)
    node = FollowerPassthroughDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

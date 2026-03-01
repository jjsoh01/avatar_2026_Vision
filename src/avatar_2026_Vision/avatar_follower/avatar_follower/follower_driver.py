import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectory
from control_msgs.action import FollowJointTrajectory

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
            JointTrajectory,
            '/leader/joint_trajectory',
            self.leader_callback,
            10
        )

        self.get_logger().info('Follower Passthrough Driver Node has been started.')

    def leader_callback(self, msg: JointTrajectory):
        # 액션 서버가 준비되었는지 확인
        if not self._action_client.wait_for_server(timeout_sec=0.1):
            self.get_logger().warn('Action server not available yet. Waiting...')
            return

        # 액션 목표(Goal) 생성 및 데이터 복사
        goal_msg = FollowJointTrajectory.Goal()
        
        # 리더로부터 받은 JointTrajectory 메시지를 액션의 trajectory 필드에 그대로 대입
        goal_msg.trajectory = msg

        # 비동기적으로 목표 전송
        self._action_client.send_goal_async(goal_msg)

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
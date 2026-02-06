#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState


class TrajToJointState(Node):
    """
    Subscribe:  JointTrajectory (default: /arm_controller/joint_trajectory)
    Publish:    JointState     (default: /joint_states)

    - RViz RobotModel은 /joint_states를 받아 URDF 관절을 갱신한다.
    - 마지막 trajectory point의 positions를 그대로 /joint_states로 변환해 퍼블리시한다.
    """

    def __init__(self):
        super().__init__('traj_to_jointstate')

        # parameters
        self.declare_parameter('traj_topic', '/arm_controller/joint_trajectory')
        self.declare_parameter('joint_states_topic', '/joint_states')

        traj_topic = self.get_parameter('traj_topic').get_parameter_value().string_value
        js_topic = self.get_parameter('joint_states_topic').get_parameter_value().string_value

        self.pub = self.create_publisher(JointState, js_topic, 10)
        self.sub = self.create_subscription(JointTrajectory, traj_topic, self.cb, 10)

        self.get_logger().info(f"Bridge ON: {traj_topic}  ->  {js_topic}")

    def cb(self, msg: JointTrajectory):
        if not msg.joint_names or not msg.points:
            return

        p = msg.points[-1]  # simplest: use last point as current target

        if len(p.positions) != len(msg.joint_names):
            self.get_logger().warn(
                f"positions({len(p.positions)}) != joint_names({len(msg.joint_names)})"
            )
            return

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = list(msg.joint_names)
        js.position = list(p.positions)

        self.pub.publish(js)


def main():
    rclpy.init()
    node = TrajToJointState()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

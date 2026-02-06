import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math

class HeadController(Node):
    def __init__(self):
        super().__init__('head_control_node')

        # ===== 설정 파라미터 =====
        self.img_w = 640.0
        self.img_h = 480.0
        self.center_x = self.img_w / 2.0
        self.center_y = self.img_h / 2.0

        # 관절 이름 (YAML 파일에 있는 이름과 정확히 일치해야 함)
        self.joint_names = ['neck_joint1', 'neck_joint2'] 
        
        # 현재 관절 각도 (초기값 0.0)
        self.pan_angle = 0.0   # neck_joint1 (좌우)
        self.tilt_angle = 0.0  # neck_joint2 (상하)

        # 관절 가동 범위 제한 (라디안 단위, 기구학적 한계에 맞춰 수정 필요)
        self.PAN_LIMIT = 1.0   # 약 57도
        self.TILT_LIMIT = 0.5  # 약 28도

        # P제어 게인 (반응 속도 조절: 클수록 빠르지만 떨림 발생 가능)
        # 방향이 반대라면 음수로 바꾸세요 (예: -0.001)
        self.pan_p_gain = 0.001 
        self.tilt_p_gain = 0.001 

        # 데드존 (중앙 근처에서는 미세하게 움직이지 않도록 함)
        self.deadzone = 20.0 

        # ===== 통신 설정 =====
        # 3번 노드에서 좌표 구독
        self.sub_cxcy = self.create_subscription(
            Float32MultiArray,
            '/yolo_deepsort/cxcy',
            self.listener_callback,
            10
        )

        # 컨트롤러에 관절 명령 퍼블리시
        self.pub_traj = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            10
        )

        self.get_logger().info("Head Controller Started. Waiting for cx, cy...")

    def listener_callback(self, msg):
        if not msg.data:
            return

        # 가장 첫 번째 객체의 좌표만 사용 (여러 개일 경우 첫 번째만 추적)
        cx = msg.data[0]
        cy = msg.data[1]

        # 1. 오차 계산 (화면 중앙 - 현재 객체 위치)
        error_x = self.center_x - cx
        error_y = self.center_y - cy

        # 2. 데드존 체크 (오차가 작으면 무시해서 모터 떨림 방지)
        if abs(error_x) < self.deadzone: error_x = 0.0
        if abs(error_y) < self.deadzone: error_y = 0.0

        if error_x == 0.0 and error_y == 0.0:
            return # 움직일 필요 없음

        # 3. 목표 각도 업데이트 (P 제어: 현재 각도 + (오차 * 게인))
        # neck_joint1 (Pan): X축 오차 대응
        self.pan_angle += (error_x * self.pan_p_gain)
        
        # neck_joint2 (Tilt): Y축 오차 대응
        self.tilt_angle += (error_y * self.tilt_p_gain)

        # 4. 각도 제한 (Safety)
        self.pan_angle = max(min(self.pan_angle, self.PAN_LIMIT), -self.PAN_LIMIT)
        self.tilt_angle = max(min(self.tilt_angle, self.TILT_LIMIT), -self.TILT_LIMIT)

        # 5. 명령 메시지 생성 및 전송
        traj_msg = JointTrajectory()
        traj_msg.header.stamp = self.get_clock().now().to_msg()
        traj_msg.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = [self.tilt_angle, self.pan_angle] #joint1이 tilt, joint2가 pan
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 100000000 # 0.1초 안에 도달해라 (반응성)

        traj_msg.points.append(point)
        self.pub_traj.publish(traj_msg)

        # 디버깅용 로그 (너무 자주 뜨면 주석 처리)
        # self.get_logger().info(f"Target: Pan={self.pan_angle:.3f}, Tilt={self.tilt_angle:.3f}")

def main(args=None):
    rclpy.init(args=args)
    node = HeadController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
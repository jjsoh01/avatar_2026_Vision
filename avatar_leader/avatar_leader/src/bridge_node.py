import math
import time
from typing import List, Dict

from dynamixel_sdk import PortHandler, PacketHandler, GroupSyncRead

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from avatar_leader.plugins.filters import LowPassFilter

# 받아올 데이터의 크기 관리
ADDR_PRESENT_POSITION = 132
LEN_PRESENT_POSITION = 4
ADDR_TORQUE_ENABLE = 64
LEN_TORQUE_ENABLE = 1

# 모터는 현재 값을 불러올 때 0 - 1023 값을 0 - 300 도 사이로 받아옴.
# 모터의 튀어나온 부분이 150도 512인 것으로 보아 여기에도 보정이 필요해보임
# https://emanual.robotis.com/docs/kr/dxl/ax/ax-12a/ 
# 위 링크의 4.19 Present Position 참고
def raw_angle_to_rad(raw_angle):
    return float(raw_angle - 1600) / 4096.0 * 2 * math.pi

class BridgeNode(Node):

    def __init__(self):
        super().__init__('bridge_node')
        # 포트 정보
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        # 통신 속도
        self.declare_parameter('serial_baud', 1000000)
        # 다이나믹셀 id 설정
        self.declare_parameter('left_ids', [1,2,3,4,5,6,7])
        self.declare_parameter('right_ids', [8,9,10,11,12,13,14])

        # 받을 주파수 
        self.declare_parameter('hz', 50.0)
        # LowPassFilter 주파수
        self.declare_parameter('lpf_cutoff_hz', 5.0)

        port = self.get_parameter('serial_port').value
        baud = int(self.get_parameter('serial_baud').value)
        hz = float(self.get_parameter('hz').value)
        cutoff = float(self.get_parameter('lpf_cutoff_hz').value)

        self.joint_names: List[str] = ["joint_L1", "joint_L2", "joint_L3", "joint_L4", "joint_L5", "joint_L6", "joint_L7",
                                       "joint_R1", "joint_R2", "joint_R3", "joint_R4", "joint_R5", "joint_R6", "joint_R7"]
        #self.joint_names: List[str] = ["test_1", "test_2", "test_3", "test_4", "test_5"]
        self.joint_ids: List[int] = [1,2,3,4,5,6,7,8,9,10,11,12,13,14]
        #self.joint_ids: List[int] = [1,2,3,4,5]

        # 다이나믹셀 자체 sdk 속 포트 핸들러 패킷 핸들러 이용
        # https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/sample_code/python_read_write_protocol_1_0/
        # 위 사이트에 자세한 사항 있으니 참고 바람
        self.port_handler = PortHandler(port)
        self.packet_handler = PacketHandler(2.0)

        self.group_sync_read = GroupSyncRead(
            self.port_handler,
            self.packet_handler,
            ADDR_PRESENT_POSITION,
            LEN_PRESENT_POSITION
        )

        self.pub = self.create_publisher(JointState, '/joint_states', 10)

        if not self.port_handler.openPort():
            raise RuntimeError(f"Failed to open port: {port}")
        if not self.port_handler.setBaudRate(baud):
            raise RuntimeError(f"Failed to set baudrate: {baud}")

        # 싱크 안정성 검사
        for dxl_id in self.joint_ids:
            ok = self.group_sync_read.addParam(dxl_id)
            if not ok:
                raise RuntimeError(f"GroupSyncRead addParam Failed for ID = {dxl_id}")
        
        # 토크 꺼졌나 확인
        if not self.torque_off_all():
           raise RuntimeError("Failed to turn torque off for all motors")
        
        self.lpf: Dict[int, LowPassFilter] = {}
        for dxl_id in self.joint_ids:
            self.lpf[dxl_id] = LowPassFilter(cutoff_hz=cutoff)

        self.dt = 1.0 / hz
        self.timer = self.create_timer(self.dt, self.tick)

        self.get_logger().info(
            f"Configured Joints: {len(self.joint_ids)})"
            f"LPF cutoff_hz = {cutoff} Hz, dt = {self.dt:.6f}s")

    def tick(self):
        comm_result = self.group_sync_read.txRxPacket()
        if comm_result != 0:
            self.get_logger().warn(f"GroupSyncRead txRxPacket failed: comm_result = {comm_result}")
            return

        positions: List[float] = []
        fail = False

        for name, dxl_id in zip(self.joint_names, self.joint_ids):
            available = self.group_sync_read.isAvailable(
                dxl_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION
            )
            if not available:
                self.get_logger().warn(f"Data not available for {name} ID = {dxl_id}")
                positions.append(float('nan'))
                fail = True
                continue

            raw = self.group_sync_read.getData(
                dxl_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION
            )
            rad = raw_angle_to_rad(int(raw))

            rad_f = self.lpf[dxl_id].step(rad, self.dt)
            positions.append(float(rad_f))

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = positions
        self.pub.publish(msg)

        if fail:
            self.get_logger().warn(
                f"/joint_states published with NaN for some joints"
                )
        else:
            self.get_logger().info(
                f"[DXL] {self.joint_names[0]} pos={positions[0]:.4f} rad (filtered)"
                )

    # 토크 자동으로 꺼주는 함수 코드
    def torque_off_all(self) -> bool:
        ok_all = True
        for dxl_id in self.joint_ids:
            comm_result, error = self.packet_handler.write1ByteTxRx(
                self.port_handler, dxl_id, ADDR_TORQUE_ENABLE, 0
                )
            if comm_result != 0 or error != 0:
                self.get_logger().warn(f"Torque off failed: comm={comm_result}, error={error}")
                return False
                ok_all = False
                continue
            
            val, comm_result, error = self.packet_handler.read1ByteTxRx(
                self.port_handler, dxl_id, ADDR_TORQUE_ENABLE
            )
            if not (comm_result == 0 and error == 0 and val == 0):
                self.get_logger().warn(
                    f"Torque Enable failed for ID = {dxl_id}: value = {val}, comm = {comm_result}, error = {error}"
                    )
                ok_all = False
        
        self.get_logger().info("Torque all OFF")
        return ok_all

# 메인은 그냥 늘 하는것
def main():
    rclpy.init()
    node = BridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
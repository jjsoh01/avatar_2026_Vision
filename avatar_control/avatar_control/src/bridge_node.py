import math
import time

from dynamixel_sdk import PortHandler, PacketHandler

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

# 받아올 데이터의 크기 관리
ADDR_PRESENT_POSITION = 36
LEN_PRESENT_POSITION = 2

# 모터는 현재 값을 불러올 때 0 - 1023 값을 0 - 300 도 사이로 받아옴.
# 모터의 튀어나온 부분이 150도 512인 것으로 보아 여기에도 보정이 필요해보임
# https://emanual.robotis.com/docs/kr/dxl/ax/ax-12a/ 
# 위 링크의 4.19 Present Position 참고
def raw_angle_to_rad(raw_angle):
    return float(raw_angle) * 300.0 / 1023.0 * math.pi / 180.0

class BridgeNode(Node):
    def __init__(self):
        super().__init__('bridge_node')

        # 포트 정보
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        # 통신 속도
        self.declare_parameter('serial_baud', 1000000)
        # 다이나믹셀 id 설정
        self.declare_parameter('dxl_id', 1)
        # 
        self.declare_parameter('topic', '/avatar/test')
        
        self.declare_parameter('hz', 50.0)

        port = self.get_parameter('serial_port').value
        baud = int(self.get_parameter('serial_baud').value)
        self.dxl_id = int(self.get_parameter('dxl_id').value)
        topic = self.get_parameter('topic').value
        hz = float(self.get_parameter('hz').value)

        self.pub = self.create_publisher(Float64, topic, 10)

        # 다이나믹셀 자체 sdk 속 포트 핸들러 패킷 핸들러 이용
        # https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/sample_code/python_read_write_protocol_1_0/
        # 위 사이트에 자세한 사항 있으니 참고 바람
        self.port_handler = PortHandler(port)
        self.packet_handler = PacketHandler(1.0)

        if self.port_handler.openPort():
            print("Succeeded to open the port!")
        else:
            raise RuntimeError(f'Falied to open port: {port}')
        
        if self.port_handler.setBaudRate(baud):
            print("Succeeded to change the baudrate!")
        else:
            raise RuntimeError(f'Falied to set baudrate: {baud}')

        self.dt = 1.0 / hz
        self.timer = self.create_timer(self.dt, self.tick)

    def tick(self):
        # https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/api_reference/python/python_packethandler/
        raw_angle, comm_result, error = self.packet_handler.read2ByteTxRx(self.port_handler, self.dxl_id, ADDR_PRESENT_POSITION)
        
        # 안정성 관련 부분
        if comm_result != 0 or error!= 0:
            self.get_logger().warn(f"DXL read failed: comm = {comm_result}")
            return
         
        rad_angle = raw_angle_to_rad(int(raw_angle))

        msg = Float64()
        msg.data = float(rad_angle)
        self.pub.publish(msg)


# 메인은 그냥 늘 하는것
def main():
    rclpy.init()
    node = BridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
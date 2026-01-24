import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
import cv2
import mediapipe as mp
from cv_bridge import CvBridge

class GazeTrackingNode(Node):
    def __init__(self):
        super().__init__('gaze_tracking_node')
        self.publisher_img = self.create_publisher(Image, 'image_raw', 10)
        self.publisher_gaze = self.create_publisher(Point, 'gaze_point', 10)
        
        # 0.033(30fps) 대신 0.05(20fps) 정도로 조정하면 CPU 부담이 줄어 끊김이 덜할 수 있습니다.
        self.timer = self.create_timer(0.05, self.timer_callback)
        
        # DroidCam 버퍼 사이즈 조정 (멈춤 현상 완화)
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1) 
        
        self.bridge = CvBridge()
        self.mp_face_mesh = mp.solutions.face_mesh
        self.face_mesh = self.mp_face_mesh.FaceMesh(
            max_num_faces=1,
            refine_landmarks=True,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )

    def timer_callback(self):
        # 버퍼를 비우기 위해 프레임을 여러 번 읽어 최신 것만 사용 (중요)
        for _ in range(2): self.cap.grab()
        ret, frame = self.cap.read()
        
        if not ret: return

        # 연산량 감소를 위해 영상 크기 축소 (성능 최적화 핵심)
        frame = cv2.resize(frame, (480, 360)) 
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.face_mesh.process(rgb_frame)

        if results.multi_face_landmarks:
            mesh_coords = results.multi_face_landmarks[0].landmark
            h, w, _ = frame.shape

            # 양쪽 눈동자 중심 추출 (468: 왼쪽, 473: 오른쪽)
            indices = [468, 473]
            for idx in indices:
                pt = mesh_coords[idx]
                cx, cy = int(pt.x * w), int(pt.y * h)
                cv2.circle(frame, (cx, cy), 4, (0, 255, 0), -1)

            # 평균값으로 시선 데이터 발행
            avg_x = (mesh_coords[468].x + mesh_coords[473].x) / 2
            avg_y = (mesh_coords[468].y + mesh_coords[473].y) / 2
            
            gaze_msg = Point()
            gaze_msg.x = avg_x - 0.5
            gaze_msg.y = avg_y - 0.5
            self.publisher_gaze.publish(gaze_msg)

        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.publisher_img.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = GazeTrackingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.cap.release()
    node.destroy_node()
    rclpy.shutdown()
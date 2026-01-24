import rclpy
from rclpy.node import Node
import cv2
from ultralytics import YOLO

class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector_node')
        # 모델 로드 (자동 다운로드됨)
        self.model = YOLO('yolov8n.pt') 
        
        # DroidCam 인덱스 (0 또는 1, 2 중 확인된 번호)
        self.cap = cv2.VideoCapture(0) 
        
        # 타이머 설정 (약 30 FPS)
        self.timer = self.create_timer(0.033, self.timer_callback)
        self.get_logger().info('YOLOv8 ROS2 Node Started!')

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        # YOLO 추론
        results = self.model(frame, verbose=False)
        
        # 결과 시각화 (Bounding Boxes)
        annotated_frame = results[0].plot()
        
        # 화면 출력
        cv2.imshow("YOLOv8 + DroidCam Detection", annotated_frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()
import rclpy
from rclpy.node import Node

from vision_msgs.msg import Detection2DArray
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float32MultiArray

# 실행 명령어
# ros2 run avatar_vision det_cxcy_node --ros-args -p target_id:=3
class YoloDeepSortDetPrinter(Node):
    def __init__(self):
        super().__init__("yolo_deepsort_det_printer")

        # ===== 파라미터 선언 =====
        # -1이면 전체 출력, 특정 숫자면 그 track_id만 출력
        self.declare_parameter("target_id", -1)
        self.target_id = self.get_parameter("target_id").value

        self.latest_track_ids = []

        self.det_sub = self.create_subscription(
            Detection2DArray,
            "/yolo_deepsort/detections",
            self.det_callback,
            10
        )

        self.id_sub = self.create_subscription(
            Int32MultiArray,
            "/yolo_deepsort/track_ids",
            self.id_callback,
            10
        )

        self.cxcy_pub = self.create_publisher(
            Float32MultiArray,
            "/yolo_deepsort/cxcy",
            10
        )


        

        self.get_logger().info(f"target_id = {self.target_id}")

    def id_callback(self, msg):
        self.latest_track_ids = list(msg.data)

    def det_callback(self, msg):
        n = len(msg.detections)

        self.get_logger().info(f"\n=== detections: {n}개 ===")


        cxcy_list = []
        
        for i, det in enumerate(msg.detections):

            # track_id 매칭
            track_id = None
            if i < len(self.latest_track_ids):
                track_id = self.latest_track_ids[i]

            # target_id 필터
            if self.target_id != -1:
                if track_id is None or track_id != self.target_id:
                    continue

            cx = det.bbox.center.position.x
            cy = det.bbox.center.position.y
            w = det.bbox.size_x
            h = det.bbox.size_y

            cxcy_list.extend([float(cx), float(cy)])
            if len(det.results) > 0:
                hyp = det.results[0].hypothesis
                class_id = hyp.class_id
                score = hyp.score
            else:
                class_id = "N/A"
                score = 0.0

            self.get_logger().info(
                f"[{i}] track_id={track_id}, class={class_id}, "
                f"score={score:.3f}, bbox(cx={cx:.1f}, cy={cy:.1f}, w={w:.1f}, h={h:.1f})"
            )
        # cxcy 퍼블리시
        cxcy_msg = Float32MultiArray()
        cxcy_msg.data = cxcy_list
        self.cxcy_pub.publish(cxcy_msg)        


def main():
    rclpy.init()
    node = YoloDeepSortDetPrinter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32, Bool

import cv2
import numpy as np
import mediapipe as mp

# ===== RealSense optional =====
try:
    import pyrealsense2 as rs
    REALSENSE_AVAILABLE = True
except ImportError:
    REALSENSE_AVAILABLE = False


class Camera:
    def __init__(self, width=640, height=480, fps=30):
        self.use_realsense = False
        self.depth_available = False

        if REALSENSE_AVAILABLE:
            try:
                self.pipeline = rs.pipeline()
                self.config = rs.config()
                self.config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
                self.config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
                self.pipeline.start(self.config)
                self.align = rs.align(rs.stream.color)

                self.use_realsense = True
                self.depth_available = True
                print("[INFO] RealSense connected")

            except Exception as e:
                print("[WARN] RealSense not found, fallback to webcam:", e)

        if not self.use_realsense:
            self.cap = cv2.VideoCapture(0)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            print("[INFO] Webcam mode")

    def read(self):
        if self.use_realsense:
            frames = self.pipeline.wait_for_frames()
            frames = self.align.process(frames)

            color = frames.get_color_frame()
            depth = frames.get_depth_frame()
            if not color:
                return False, None, None

            return True, np.asanyarray(color.get_data()), depth
        else:
            ret, frame = self.cap.read()
            return ret, frame, None

    def release(self):
        if self.use_realsense:
            self.pipeline.stop()
        else:
            self.cap.release()


class FacePublisher(Node):
    def __init__(self):
        super().__init__('face_publisher')

        self.rgb_pub = self.create_publisher(
            CompressedImage,
            '/camera/color/image_raw/compressed',
            10
        )
        self.dist_pub = self.create_publisher(Float32, '/face/distance_m', 10)
        self.approach_pub = self.create_publisher(Bool, '/face/is_approaching', 10)
        self.gaze_pub = self.create_publisher(Bool, '/face/is_looking', 10)

        self.cam = Camera()

        self.face_mesh = mp.solutions.face_mesh.FaceMesh(
            max_num_faces=1,
            refine_landmarks=True
        )

        self.prev_distance = None
        self.timer = self.create_timer(1.0 / 30.0, self.timer_cb)

        self.get_logger().info("Face publisher started (RealSense / Webcam auto)")

    def timer_cb(self):
        ret, frame, depth = self.cam.read()
        if not ret:
            return

        h, w, _ = frame.shape
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.face_mesh.process(rgb)

        if not results.multi_face_landmarks:
            return

        face = results.multi_face_landmarks[0]

        LEFT_EYE, RIGHT_EYE = 33, 263
        LEFT_IRIS, RIGHT_IRIS = 468, 473

        def px(i):
            lm = face.landmark[i]
            return int(lm.x * w), int(lm.y * h)

        le, re = px(LEFT_EYE), px(RIGHT_EYE)
        li, ri = px(LEFT_IRIS), px(RIGHT_IRIS)

        # ===== Distance (RealSense only) =====
        if self.cam.depth_available:
            cx = int((le[0] + re[0]) / 2)
            cy = int((le[1] + re[1]) / 2)

            samples = [
                depth.get_distance(cx + dx, cy + dy)
                for dx in range(-2, 3)
                for dy in range(-2, 3)
                if 0.2 < depth.get_distance(cx + dx, cy + dy) < 3.5
            ]

            if samples:
                dist = float(np.mean(samples))
                self.dist_pub.publish(Float32(data=dist))

                approaching = self.prev_distance is not None and (self.prev_distance - dist) > 0.02
                self.approach_pub.publish(Bool(data=approaching))
                self.prev_distance = dist

        # ===== Gaze =====
        eye_width = abs(re[0] - le[0])
        iris_center = (li[0] + ri[0]) / 2
        eye_center = (le[0] + re[0]) / 2
        looking = abs(iris_center - eye_center) < eye_width * 0.08
        self.gaze_pub.publish(Bool(data=looking))

        # ===== RGB publish =====
        ok, encoded = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
        if ok:
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.format = 'jpeg'
            msg.data = encoded.tobytes()
            self.rgb_pub.publish(msg)

    def destroy_node(self):
        self.cam.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FacePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

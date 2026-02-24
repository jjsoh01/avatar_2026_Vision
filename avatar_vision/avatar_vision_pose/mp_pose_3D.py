#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge

import numpy as np
import cv2
import math
import pyrealsense2 as rs

import mediapipe as mp


class MediaPipePose3DNode(Node):
    """
    - Input:
        /realsense/color/image_raw/compressed  (sensor_msgs/CompressedImage)
        /realsense/aligned_depth_to_color/image_raw (sensor_msgs/Image, 16UC1)
    - Output:
        /mediapipe/pose_points_3d (std_msgs/Float32MultiArray)
        data = [landmark_id, X, Y, Z, landmark_id, X, Y, Z, ...]  (meters, camera frame)
    """

    def __init__(self):
        super().__init__("mediapipe_pose_3d_node")

        self.bridge = CvBridge()

        # ===== Parameters (intrinsics) =====
        self.declare_parameter("fx", 615.1111450195312)
        self.declare_parameter("fy", 615.2798461914062)
        self.declare_parameter("ppx", 318.12139892578125)
        self.declare_parameter("ppy", 250.6387481689453)

        self.fx = float(self.get_parameter("fx").value)
        self.fy = float(self.get_parameter("fy").value)
        self.ppx = float(self.get_parameter("ppx").value)
        self.ppy = float(self.get_parameter("ppy").value)

        # depth raw는 16UC1 (unit: device unit)
        self.declare_parameter("depth_scale", 0.001)  # meters per unit
        self.depth_scale = float(self.get_parameter("depth_scale").value)

        # MediaPipe settings
        self.declare_parameter("min_detection_conf", 0.5)
        self.declare_parameter("min_tracking_conf", 0.5)
        self.min_det = float(self.get_parameter("min_detection_conf").value)
        self.min_trk = float(self.get_parameter("min_tracking_conf").value)

        # Which landmarks publish (기본: 팔 관련만)
        # MediaPipe PoseLandmark enum index:
        # 11 L_SHOULDER, 12 R_SHOULDER, 13 L_ELBOW, 14 R_ELBOW, 15 L_WRIST, 16 R_WRIST
        self.declare_parameter("landmark_ids", [11, 12, 13, 14, 15, 16])
        self.landmark_ids = list(self.get_parameter("landmark_ids").value)

        # Output
        self.pub = self.create_publisher(Float32MultiArray, "/mediapipe/pose_points_3d", 10)

        # Subscribers
        self.sub_rgb = self.create_subscription(
            CompressedImage,
            "/realsense/color/image_raw/compressed",
            self.rgb_cb,
            10
        )
        self.sub_depth = self.create_subscription(
            Image,
            "/realsense/aligned_depth_to_color/image_raw",
            self.depth_cb,
            10
        )

        self.latest_depth = None  # numpy uint16 (H,W)
        self.latest_depth_stamp = None

        # MediaPipe init
        self.mp_pose = mp.solutions.pose
        self.pose = self.mp_pose.Pose(
            static_image_mode=False,
            model_complexity=1,
            enable_segmentation=False,
            min_detection_confidence=self.min_det,
            min_tracking_confidence=self.min_trk
        )

        # RealSense intrinsics object for rs2_deproject
        self.intr = rs.intrinsics()
        self.intr.width = 640
        self.intr.height = 480
        self.intr.fx = self.fx
        self.intr.fy = self.fy
        self.intr.ppx = self.ppx
        self.intr.ppy = self.ppy
        self.intr.model = rs.distortion.none
        self.intr.coeffs = [0, 0, 0, 0, 0]

        self.get_logger().info(
            f"Started. fx={self.fx:.3f}, fy={self.fy:.3f}, ppx={self.ppx:.3f}, ppy={self.ppy:.3f}, depth_scale={self.depth_scale}"
        )

    def depth_cb(self, msg: Image):
        # msg.encoding should be 16UC1
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="16UC1")
            self.latest_depth = depth
            self.latest_depth_stamp = msg.header.stamp
        except Exception as e:
            self.get_logger().warn(f"Depth convert failed: {e}")

    def rgb_cb(self, msg: CompressedImage):
        if self.latest_depth is None:
            return

        # Decode compressed image
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None:
            return

        h, w = frame.shape[:2]
        depth = self.latest_depth
        if depth.shape[0] != h or depth.shape[1] != w:
            # aligned depth가 정말 color 해상도랑 같아야 정상
            self.get_logger().warn(f"Depth size {depth.shape} != RGB size {(h,w)}")
            return

        # MediaPipe expects RGB
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.pose.process(rgb)
        if not results.pose_landmarks:
            cv2.imshow("mp_pose_3d_debug", frame)
            cv2.waitKey(1)
            return

        # Build output array
        out = []
        for lm_id in self.landmark_ids:
            lm = results.pose_landmarks.landmark[lm_id]

            # normalized -> pixel
            u = int(lm.x * w)
            v = int(lm.y * h)

            # clamp
            u = max(0, min(w - 1, u))
            v = max(0, min(h - 1, v))

            # depth in meters (median in 3x3 to reduce noise)
            z_m = self.depth_median_m(depth, u, v, k=1)  # k=1 => 3x3
            if z_m is None:
                continue

            # deproject -> camera XYZ (meters)
            X, Y, Z = rs.rs2_deproject_pixel_to_point(self.intr, [float(u), float(v)], float(z_m))

            out.extend([float(lm_id), float(X), float(Y), float(Z)])

            # debug draw
            cv2.circle(frame, (u, v), 5, (0, 255, 0), -1)
            cv2.putText(frame, f"{lm_id}", (u + 6, v - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        # Publish
        msg_out = Float32MultiArray()
        msg_out.data = out
        self.pub.publish(msg_out)

        cv2.imshow("mp_pose_3d_debug", frame)
        cv2.waitKey(1)

    def depth_median_m(self, depth_u16: np.ndarray, u: int, v: int, k: int = 1):
        """
        depth_u16: 16UC1 depth image (aligned)
        u,v: pixel
        k=1 => 3x3, k=2 => 5x5
        Return depth in meters (float) or None if invalid.
        """
        h, w = depth_u16.shape[:2]
        u0 = max(0, u - k)
        u1 = min(w - 1, u + k)
        v0 = max(0, v - k)
        v1 = min(h - 1, v + k)

        window = depth_u16[v0:v1 + 1, u0:u1 + 1].astype(np.uint16).reshape(-1)

        # remove zeros
        window = window[window > 0]
        if window.size == 0:
            return None

        med = float(np.median(window))
        return med * self.depth_scale


def main(args=None):
    rclpy.init(args=args)
    node = MediaPipePose3DNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np
import pyrealsense2 as rs

class realsense_camera:
    is_opened = False
    config = None
    intr = None
    depth_scale = 0.001  # 기본값(나중에 장치에서 읽어서 갱신)

    def __init__(self, height=480, width=640, fps=30, use_color=True, use_depth=False, align_depth_to_color=True):
        self.height = height
        self.width = width
        self.fps = fps
        self.use_depth = use_depth
        self.use_color = use_color
        self.align_depth_to_color = align_depth_to_color

        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)

        # depth를 color로 정렬하는 align 객체
        self.align = rs.align(rs.stream.color) if (self.use_depth and self.align_depth_to_color) else None

        if self.can_connect():
            pipeline_profile = self.config.resolve(self.pipeline_wrapper)
            device = pipeline_profile.get_device()

            found_rgb = False
            for s in device.sensors:
                if s.get_info(rs.camera_info.name) == 'RGB Camera':
                    found_rgb = True
            if not found_rgb:
                self.use_color = False

            if self.use_color:
                self.config.enable_stream(
                    rs.stream.color, self.width, self.height, rs.format.bgr8, self.fps
                )

            if self.use_depth:
                self.config.enable_stream(
                    rs.stream.depth, self.width, self.height, rs.format.z16, self.fps
                )

            if self.use_color:
                profile = self.pipeline.start(self.config)
                self.is_opened = True

                # intrinsics (color 기준)
                self.intr = (
                    self.pipeline
                    .get_active_profile()
                    .get_stream(rs.stream.color)
                    .as_video_stream_profile()
                    .get_intrinsics()
                )

                # depth_scale 읽기 (단위: meters per depth unit)
                try:
                    depth_sensor = profile.get_device().first_depth_sensor()
                    self.depth_scale = float(depth_sensor.get_depth_scale())
                except Exception:
                    pass

    def can_connect(self):
        return self.config.can_resolve(self.pipeline_wrapper)

    def isOpened(self):
        return self.is_opened

    def read(self):
        """
        반환:
          ret, color_image, depth_image(raw), aligned_depth_image(color에 정렬)
        """
        try:
            frames = self.pipeline.wait_for_frames(100)

            color_frame = frames.get_color_frame() if self.use_color else None
            depth_frame = frames.get_depth_frame() if self.use_depth else None
            if self.use_color and not color_frame:
                return False, None, None, None

            # aligned depth 계산
            aligned_depth_frame = None
            if self.align is not None:
                aligned_frames = self.align.process(frames)
                aligned_depth_frame = aligned_frames.get_depth_frame()

            color_image = np.asanyarray(color_frame.get_data()) if color_frame else None
            depth_image = np.asanyarray(depth_frame.get_data()) if depth_frame else None
            aligned_depth_image = np.asanyarray(aligned_depth_frame.get_data()) if aligned_depth_frame else None

            return True, color_image, depth_image, aligned_depth_image
        except:
            return False, None, None, None

    def release(self):
        if self.is_opened:
            self.pipeline.stop()


class RealSenseRGBDPublisher(Node):
    def __init__(self):
        super().__init__('realsense_rgbd_publisher')

        self.bridge = CvBridge()

        # RGB compressed (기존)
        self.pub_rgb = self.create_publisher(
            CompressedImage,
            '/realsense/color/image_raw/compressed',
            10
        )

        # Depth raw
        self.pub_depth = self.create_publisher(
            Image,
            '/realsense/depth/image_raw',
            10
        )

        # Aligned depth to color (정석)
        self.pub_aligned_depth = self.create_publisher(
            Image,
            '/realsense/aligned_depth_to_color/image_raw',
            10
        )

        self.cam = realsense_camera(
            width=640,
            height=480,
            fps=30,
            use_color=True,
            use_depth=True,
            align_depth_to_color=True
        )

        if not self.cam.isOpened():
            self.get_logger().error('RealSense camera not opened')
            return

        self.get_logger().info(f"Depth scale (m/unit): {self.cam.depth_scale}")

        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)
        self.get_logger().info('RealSense RGB + Depth + AlignedDepth publisher started')

    def timer_callback(self):
        ret, color, depth, aligned_depth = self.cam.read()
        if not ret or color is None:
            return

        stamp = self.get_clock().now().to_msg()

        # ---------- RGB compressed ----------
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 70]
        success, encoded_image = cv2.imencode('.jpg', color, encode_param)
        if success:
            msg = CompressedImage()
            msg.header.stamp = stamp
            msg.header.frame_id = 'realsense_color_frame'
            msg.format = 'jpeg'
            msg.data = encoded_image.tobytes()
            self.pub_rgb.publish(msg)

        # ---------- Depth raw ----------
        if depth is not None:
            msg_d = self.bridge.cv2_to_imgmsg(depth, encoding='16UC1')
            msg_d.header.stamp = stamp
            msg_d.header.frame_id = 'realsense_depth_frame'
            self.pub_depth.publish(msg_d)

        # ---------- Aligned depth ----------
        if aligned_depth is not None:
            msg_ad = self.bridge.cv2_to_imgmsg(aligned_depth, encoding='16UC1')
            msg_ad.header.stamp = stamp
            msg_ad.header.frame_id = 'realsense_color_frame'
            self.pub_aligned_depth.publish(msg_ad)

        # 디버그 화면
        cv2.imshow('RGB', color)
        if aligned_depth is not None:
            depth_vis = cv2.normalize(aligned_depth, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
            cv2.imshow('Aligned Depth (vis)', depth_vis)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = RealSenseRGBDPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.cam.release()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    
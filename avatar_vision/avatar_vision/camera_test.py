import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
# 압축해서 보내기 위해
from sensor_msgs.msg import CompressedImage


# OpenCV 이미지 <-> ROS Image 변환을 위해.
from cv_bridge import CvBridge 

# OpenCV
import cv2
import numpy as np

#Intel RealSense SDK(Python)
import pyrealsense2 as rs

# RealSense를 일반 카메라처럼 쓰게 해줌
class realsense_camera:
    is_opened = False
    config = None
    intr = None

    def __init__(self, height=480, width=640, fps=30, use_color=True, use_depth=False):
        self.height = height
        self.width = width
        self.fps = fps
        self.use_depth = use_depth
        self.use_color = use_color

        self.pipeline = rs.pipeline() # RealSense 프레임을 흘려보내는 파이프라인 생성
        self.config = rs.config() # 파이프라인에 넣을 설정들
        self.pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)

        if self.can_connect(): # 연결 확인
            pipeline_profile = self.config.resolve(self.pipeline_wrapper)
            device = pipeline_profile.get_device() # RealSense 카메라에서 객체 가져오기

            found_rgb = False
            
            # RGB 센서 존재 여부 검사
            # device.sensors를 돌면서 'RGB Camera'가 있는지 확인
            # 없으면 self.use_color = False로 꺼버림
            for s in device.sensors:
                if s.get_info(rs.camera_info.name) == 'RGB Camera':
                    found_rgb = True

            if not found_rgb:
                self.use_color = False

            if self.use_color:
                self.config.enable_stream(
                    rs.stream.color,
                    self.width,
                    self.height,
                    rs.format.bgr8,
                    self.fps
                )

            if self.use_color:
                self.pipeline.start(self.config) # 영상 스트리밍 시작
                self.is_opened = True
                self.intr = (
                    self.pipeline
                    .get_active_profile()
                    .get_stream(rs.stream.color)
                    .as_video_stream_profile()
                    .get_intrinsics()
                )

    def can_connect(self):
        return self.config.can_resolve(self.pipeline_wrapper)

    def isOpened(self):
        return self.is_opened

    # 프레임을 1장 받아서 numpy 이미지로 반환
    def read(self):
        try:
            frames = self.pipeline.wait_for_frames(100)
            color_frame = frames.get_color_frame() # frameset에서 컬러 프레임 추출
            if not color_frame:
                return False, None
            color_image = np.asanyarray(color_frame.get_data())
            return True, color_image
        except:
            return False, None

    def release(self):
        if self.is_opened:
            self.pipeline.stop()

# Publisher node 생성
class RealSenseRGBPublisher(Node):

    def __init__(self):
        super().__init__('realsense_rgb_publisher') # 노드 이름

        self.publisher = self.create_publisher(
            CompressedImage, # 메시지 자료형
            '/realsense/color/image_raw/compressed', # 토픽 이름
            10
        )


        self.cam = realsense_camera(
            width=640,
            height=480,
            fps=30,
            use_color=True,
            use_depth=False
        )

        if not self.cam.isOpened():
            self.get_logger().error('RealSense camera not opened')
            return

        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)
        self.get_logger().info('RealSense RGB publisher started')

# Publish 하면 할 일
    def timer_callback(self):
        ret, frame = self.cam.read() # 프레임 1개 읽어와서
        if not ret:
            return

        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 70]
        success, encoded_image = cv2.imencode('.jpg', frame, encode_param)
        if not success:
            return

        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg() # 타임 스탬프 찍고
        msg.header.frame_id = 'realsense_color_frame' # 이름 설정
        msg.format = 'jpeg'
        msg.data = encoded_image.tobytes()

        self.publisher.publish(msg) # 메시지 발행

        # 로컬 화면 출력용
        cv2.imshow('RealSense RGB Publisher View', frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = RealSenseRGBPublisher()
    rclpy.spin(node)
    node.cam.release()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


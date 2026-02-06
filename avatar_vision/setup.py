from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'avatar_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
    ],
    install_requires=[
        'setuptools',
        'numpy<2.0',          # ✅ mediapipe 안전선
        'opencv-python',      # subscriber / publisher 공용
    ],
    zip_safe=True,
    maintainer='dongryun',
    maintainer_email='storm5030@gmail.com',
    description='Camera-agnostic face perception (RealSense / Webcam)',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
<<<<<<< HEAD
            'realsense_face_publisher = avatar_vision.realsense_face_publisher:main',
            'realsense_rgb_subscriber = avatar_vision.realsense_rgb_subscriber:main',
=======
            'rgb_subscriber = avatar_vision.rgb_sub:main',
            'realsense_rgb_publisher = avatar_vision.camera_test:main',
            'hand_node = avatar_vision.hand_node:main',
            'tracker_node = avatar_vision.tracker_node:main',
            'webcam_publisher = avatar_vision.webcam_publisher:main',
            'yolo_deepsort_subscriber = avatar_vision.webcam_pub_and_tracker:main',
            'det_cxcy_node = avatar_vision.det_cxcy_node:main',
>>>>>>> d485cd7bb458faa5e916b011bfee47acb4b780ee
        ],
    },
)

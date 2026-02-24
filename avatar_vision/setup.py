from setuptools import find_packages, setup

package_name = 'avatar_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dongryun',
    maintainer_email='storm5030@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'rgb_subscriber = avatar_vision.rgb_sub:main',
            'realsense_rgb_publisher = avatar_vision.camera_pub:main',
            'tracker_node = avatar_vision.tracker_node:main',
            'hand_tracker = avatar_vision.hand_tracker_node:main',
            'angle_pub = avatar_vision.angle_pub:main',
            'depth_pub = avatar_vision_pose.camera_depth_pub:main',
            'mp_pose_3d = avatar_vision_pose.mp_pose_3D:main',
            'webcam_publisher = avatar_vision_etc.webcam_publisher:main',
            'yolo_deepsort_subscriber = avatar_vision_etc.webcam_pub_and_tracker:main',
            'hand_node = avatar_vision_etc.hand_node:main',
        ],
    },
)

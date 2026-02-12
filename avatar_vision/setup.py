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
            'realsense_rgb_publisher = avatar_vision.camera_test:main',
            'webcam_pub = avatar_vision.webcam_pub:main',
            'hand_node = avatar_vision.hand_node:main',
            'tracker_node = avatar_vision.tracker_node:main',
            'webcam_publisher = avatar_vision.webcam_publisher:main',
            'yolo_deepsort_subscriber = avatar_vision.webcam_pub_and_tracker:main',
            'det_cxcy_node = avatar_vision.det_cxcy_node:main',
            'head_control_node = avatar_vision.head_control_node:main',
            'pose_estimation_node = avatar_vision.pose_estimation_node:main',
        ],
    },
)

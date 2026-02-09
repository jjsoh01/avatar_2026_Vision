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
            'webcam_publisher = avatar_vision.camera_test_webcam:main',
            'hand_tracking_node = avatar_vision.hand_node:main',
            'yolo_node = avatar_vision.yolo_node:main',
            'droidcam_publisher = my_camera_package.droidcam_pub:main',
            'gaze_subscriber = my_camera_package.gaze_sub:main',
            'face_direction_node = avatar_vision.face_direction:main',
            'face_attention_node = avatar_vision.face_attention:main'
        ],
    },
)

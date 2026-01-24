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
            'braille_reader = avatar_vision.braille_node:main',
            'braille_listener = avatar_vision.braille_listener:main',
            'rgb_subscriber = avatar_vision.rgb_sub:main',
            'realsense_rgb_publisher = avatar_vision.camera_test:main',
            'webcam_publisher = avatar_vision.webcam_publisher:main',
        ],
    },
)

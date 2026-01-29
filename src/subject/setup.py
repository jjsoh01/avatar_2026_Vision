from setuptools import find_packages, setup

package_name = 'subject'

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
    maintainer='ohseonggeun',
    maintainer_email='dhtjdrms6924@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # 얼굴 트래킹 관련 노드
            'face_node = subject.face.FaceGaze_node:main',
            'face_sub_node = subject.face.FaceGaze_sub_node:main',
            
            # 손 트래킹 관련 노드
            'hand_node = subject.hand.hand_node:main',
            'hand_sub_node = subject.hand.hand_sub_node:main',
        ],
    },
)

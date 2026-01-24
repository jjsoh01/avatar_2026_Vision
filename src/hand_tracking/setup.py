from setuptools import find_packages, setup

package_name = 'hand_tracking'

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
            'hand_node = hand_tracking.hand_node:main',
            'hand_sub_node = hand_tracking.hand_sub_node:main',
        ],
    },
)

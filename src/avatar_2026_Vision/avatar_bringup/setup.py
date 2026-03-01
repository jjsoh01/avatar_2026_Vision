from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'avatar_bringup'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),

        ('share/' + package_name, ['package.xml']),
        
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        
        (os.path.join('share', package_name, 'config/follower'), glob('config/follower/*.yaml')),
        (os.path.join('share', package_name, 'config/leader'), glob('config/leader/*.yaml')),
        (os.path.join('share', package_name, 'config/vision'), glob('config/vision/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dongryun',
    maintainer_email='storm5030@gmail.com',
    description='Bringup package for Avatar Robot (Python)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_trajectory_executor = avatar_bringup.joint_trajectory_executor:main',
            'traj_to_jointstate = avatar_bringup.traj_to_jointstate:main',
        ],
    },
)
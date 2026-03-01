#!/usr/bin/env python3
#
# Copyright 2024 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Sungho Woo, Woojin Wie, Junha Cha

"""Launch file for AI teleoperation of OMX L for leader, OMX F for follower."""

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import LogInfo
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.event_handlers import OnProcessStart
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for AI teleoperation."""
    # 팔로워 런치파일 실행
    # Step 1: Start follower launch file
    start_follower = ExecuteProcess(
        cmd=[
            'ros2',
            'launch',
            'avatar_bringup',
            'follower.launch.py',
        ],
        output='screen',
    )

    # 팔로워의 초기 위치 가져옴
    # Step 2: Run the joint trajectory executor for the follower
    trajectory_params_file = PathJoinSubstitution([
        FindPackageShare('avatar_bringup'),
        'config',
        'follower',
        'initial_positions.yaml',
    ])

    # 팔로워 초기 위치 작동 시, 궤적 생성기
    joint_trajectory_executor = Node(
        package='avatar_bringup',
        executable='joint_trajectory_executor',
        parameters=[trajectory_params_file],
        output='screen',
    )

    # 리더 런치파일 실행
    # Step 3: Start leader launch file
    start_leader = ExecuteProcess(
        cmd=[
            'ros2',
            'launch',
            'avatar_bringup',
            'leader.launch.py',
        ],
        output='screen',
        shell=True,
    )

    return LaunchDescription([
        LogInfo(msg='Starting follower.launch.py...'),
        start_follower,
        # Step 2: Ensure joint_trajectory_executor starts after start_follower
        RegisterEventHandler(
            OnProcessStart(
                target_action=start_follower,
                on_start=[
                    LogInfo(
                        msg=(
                            '✅ follower.launch.py has fully started. '
                            'Running joint_trajectory_executor...'
                        )
                    ),
                    joint_trajectory_executor,
                ],
            )
        ),
        # Step 3: Ensure start_leader starts after joint_trajectory_executor
        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_trajectory_executor,
                on_exit=[
                    LogInfo(
                        msg=(
                            '✅ joint_trajectory_executor has completed. '
                            'Starting leader.launch.py...'
                        )
                    ),
                    start_leader,
                ],
            )
        ),
    ])

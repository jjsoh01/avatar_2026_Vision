from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_share = FindPackageShare('avatar_bringup')
    pkg_description = FindPackageShare('avatar_description')

    xacro_file = PathJoinSubstitution([pkg_description, 'urdf/follower', 'follower.urdf.xacro'])
    rviz_config = PathJoinSubstitution([pkg_description, 'rviz', 'avatar_model.rviz'])
    controllers_file = PathJoinSubstitution([pkg_share, 'config', 'follower', 'hardware_controller_manager.yaml'])

    robot_description_content = ParameterValue(
    Command([
        'xacro ',
        xacro_file,
        ' use_sim:=false',
        ' use_mock_hardware:=true',
        ' mock_sensor_commands:=false'
    ]),
    value_type=str
)


    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_description_content}, controllers_file],
        output='screen'
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller'],
    )

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_content}]
        ),

        control_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        )
    ])

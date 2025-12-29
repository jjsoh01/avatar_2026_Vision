from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    serial_port = LaunchConfiguration('serial_port')
    serial_baud = LaunchConfiguration('serial_baud')
    dxl_id = LaunchConfiguration('dxl_id')
    topic = LaunchConfiguration('topic')
    hz = LaunchConfiguration('hz')

    param_file = PathJoinSubstitution([
        FindPackageShare('avatar_bringup'),
        'config',
        'leader.yaml'
    ])

    bridge_node = Node(
        package='avatar_control',    
        executable='leader_bridge',       
        name='leader_bridge',
        output='screen',
        parameters=[
            param_file,
            {
                'serial_port': serial_port,
                'serial_baud': serial_baud,
                'dxl_id': dxl_id,
                'topic': topic,
                'hz': hz,
            }
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('serial_baud', default_value='1000000'),
        DeclareLaunchArgument('dxl_id', default_value='1'),
        DeclareLaunchArgument('topic', default_value='/avatar/test'),
        DeclareLaunchArgument('hz', default_value='50.0'),

        bridge_node
    ])


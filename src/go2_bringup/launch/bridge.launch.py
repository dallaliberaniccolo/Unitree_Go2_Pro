from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    config = str(Path(get_package_share_directory('go2_bringup')) / 'config' / 'bridge.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=config),
        Node(package='go2_bridge', executable='bridge', namespace='go2', name='go2_bridge',
             parameters=[LaunchConfiguration('config')], output='screen', emulate_tty=True)
    ])

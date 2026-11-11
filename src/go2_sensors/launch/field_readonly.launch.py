"""GNSS+IMU+bridge+navigation+mission: read only. NO motion!"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('sensors_config',default_value='/ws/src/go2_sensors/config/field_sensors.yaml'),
        DeclareLaunchArgument('bridge_config',default_value='/ws/config/go2_real_readonly.yaml'),
        Node(package='go2_bridge',executable='bridge',namespace='go2',name='go2_bridge',
             output='screen',parameters=[LaunchConfiguration('bridge_config')]),
        Node(package='go2_sensors',executable='gps_node',output='screen',
             parameters=[LaunchConfiguration('sensors_config')]),
        Node(package='go2_sensors',executable='imu_mag_node',output='screen',
             parameters=[LaunchConfiguration('sensors_config')]),
        Node(package='go2_navigation',executable='navigation_node',output='screen',
             parameters=[{'min_satellites':7}]),
        Node(package='go2_mission_manager',executable='mission_manager_node',output='screen',
             parameters=[{'gps_loss_timeout_sec':20.0}]),
    ])

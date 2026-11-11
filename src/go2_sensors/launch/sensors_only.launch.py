"""Sensor hardware tests only. No robot bridge, no movement."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default='/ws/src/go2_sensors/config/field_sensors.yaml'
    return LaunchDescription([
        DeclareLaunchArgument('sensors_config',default_value=default),
        Node(package='go2_sensors',executable='gps_node',output='screen',
             parameters=[LaunchConfiguration('sensors_config')]),
        Node(package='go2_sensors',executable='imu_mag_node',output='screen',
             parameters=[LaunchConfiguration('sensors_config')]),
    ])

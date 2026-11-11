"""ROS2 Jazzy read-only navigation & mission supervisor. Never outputs commands."""
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='go2_navigation', executable='navigation_node',
             output='screen', parameters=[{'min_satellites': 7}]),
        Node(package='go2_mission_manager', executable='mission_manager_node', output='screen',
             parameters=[{'gps_loss_timeout_sec': 20.0}]),
    ])

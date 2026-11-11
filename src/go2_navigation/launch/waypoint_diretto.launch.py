# Launch UNICO: Bridge + GPS + (opzionale) IMU esterna + Navigation + Mission Manager
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    cfg = LaunchConfiguration('config')
    motion = ParameterValue(LaunchConfiguration('enable_motion'), value_type=bool)
    auto_start = ParameterValue(LaunchConfiguration('auto_start_on_goal'), value_type=bool)
    transport = LaunchConfiguration('motion_transport')
    lock_line = ParameterValue(LaunchConfiguration('lock_straight_line'), value_type=bool)

    return LaunchDescription([
        DeclareLaunchArgument('config', default_value='/ws/config/go2_waypoint_diretto.yaml'),
        DeclareLaunchArgument('enable_motion', default_value='false'),
        DeclareLaunchArgument('auto_start_on_goal', default_value='true'),
        DeclareLaunchArgument('motion_transport', default_value='sport'),
        DeclareLaunchArgument('lock_straight_line', default_value='false'),
        DeclareLaunchArgument('start_gps', default_value='true'),
        DeclareLaunchArgument('start_imu_ext', default_value='true'),
        DeclareLaunchArgument('start_detector', default_value='true'),

        Node(package='go2_sensors', executable='gps_node', name='go2_gps_sensor',
             condition=IfCondition(LaunchConfiguration('start_gps')),
             output='screen', respawn=True, respawn_delay=5.0, parameters=[cfg]),

        Node(package='go2_sensors', executable='imu_mag_node', name='go2_imu_mag_sensor',
             condition=IfCondition(LaunchConfiguration('start_imu_ext')),
             output='screen', respawn=True, respawn_delay=5.0, parameters=[cfg]),

        Node(package='go2_animal_detector', executable='animal_detector_node', name='go2_animal_detector',
             condition=IfCondition(LaunchConfiguration('start_detector')),
             output='screen', respawn=True, respawn_delay=5.0, parameters=[cfg]),

        TimerAction(period=3.0, actions=[Node(package='go2_navigation', executable='navigation_node', name='go2_navigation',
             output='screen', respawn=True, respawn_delay=5.0, parameters=[cfg])]),
        TimerAction(period=5.0, actions=[Node(package='go2_bridge', executable='bridge', namespace='go2', name='go2_bridge',
             output='screen', respawn=True, respawn_delay=5.0,
             parameters=[cfg, {'motion_enabled': motion, 'motion_transport': transport}])]),
        TimerAction(period=7.0, actions=[Node(package='go2_mission_manager', executable='mission_manager_node', name='go2_mission_manager',
             output='screen', respawn=True, respawn_delay=5.0,
             parameters=[cfg, {'enable_motion': motion, 'lock_straight_line': lock_line,
                               'auto_start_on_goal': auto_start}])]),
    ])

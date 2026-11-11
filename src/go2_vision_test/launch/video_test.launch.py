"""Real detector + real mission manager, synthetic video/navigation, no robot bridge."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from go2_vision_test.topics import MANAGER_REMAPS, DETECTOR_REMAPS


def generate_launch_description():
    def value(name, kind):
        return ParameterValue(LaunchConfiguration(name), value_type=kind)

    return LaunchDescription([
        DeclareLaunchArgument('video_path', description='Absolute path of a local video file'),
        DeclareLaunchArgument('model_path', default_value='/ws/models/wolf/wolf_detector.pt'),
        DeclareLaunchArgument('fps', default_value='1.5'),
        DeclareLaunchArgument('detector_control_topic', default_value='/vision_test/identification/control'),
        DeclareLaunchArgument('record_directory', default_value='/ws/log/wolf_samples'),
        DeclareLaunchArgument('loop', default_value='false'),
        DeclareLaunchArgument('playback_speed', default_value='1.0'),
        DeclareLaunchArgument('source_fps', default_value='0.0'),
        DeclareLaunchArgument('initial_distance_m', default_value='25.0'),
        DeclareLaunchArgument('approach_speed_mps', default_value='0.0'),
        DeclareLaunchArgument('readiness_delay_sec', default_value='0.0'),
        Node(package='go2_vision_test', executable='video_camera_node',
             name='video_test_camera', output='screen', parameters=[{
                 'video_path': value('video_path', str), 'publish_rate_hz': value('fps', float),
                 'loop': value('loop', bool), 'playback_speed': value('playback_speed', float),
                 'source_fps': value('source_fps', float), 'wait_for_detector': True}]),
        Node(package='go2_animal_detector', executable='animal_detector_node',
             name='video_test_detector', output='screen',
             remappings=[(a, b) for a, b in DETECTOR_REMAPS if a != '/identification/control']
             + [('/identification/control', LaunchConfiguration('detector_control_topic'))],
             parameters=[{'model_path': value('model_path', str),
                          'record_directory': value('record_directory', str),
                          'record_source': value('video_path', str),
                          'inference_rate_hz': value('fps', float)}]),
        Node(package='go2_mission_manager', executable='mission_manager_node',
             name='video_test_manager', output='screen', remappings=MANAGER_REMAPS,
             parameters=[{'enable_motion': False, 'auto_start_on_goal': True,
                          'sport_state_code_is_fault': False}]),
        Node(package='go2_vision_test', executable='video_test_inputs_node',
             name='video_test_inputs', output='screen', parameters=[{
                 'initial_distance_m': value('initial_distance_m', float),
                 'approach_speed_mps': value('approach_speed_mps', float),
                 'readiness_delay_sec': value('readiness_delay_sec', float)}]),
    ])

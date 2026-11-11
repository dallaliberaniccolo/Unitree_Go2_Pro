"""Synthetic localization for the video test; all endpoints are under /vision_test."""
import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from go2_interfaces.msg import BridgeState

from .topics import ROOT


class VideoTestInputs(Node):
    def __init__(self):
        super().__init__('video_test_inputs')
        defaults = dict(initial_distance_m=25.0, approach_speed_mps=0.0, readiness_delay_sec=0.0)
        for key, value in defaults.items():
            self.declare_parameter(key, value, ParameterDescriptor(read_only=True))
        cfg = {key: self.get_parameter(key).value for key in defaults}
        if any(not math.isfinite(v) or v < 0 for v in cfg.values()):
            raise ValueError('Synthetic distance, speed and delay must be non-negative')
        self.distance = cfg['initial_distance_m']
        self.speed = cfg['approach_speed_mps']
        self.ready_delay = cfg['readiness_delay_sec']
        self.started = self.last_tick = time.monotonic()
        self.last_goal_sent = None
        self.goal = [45.0, 11.0]
        self.goal_accepted = False
        self.manager_state = 'IDLE'
        self.gps_updates = 0
        control_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.VOLATILE)
        self.goal_pub = self.create_publisher(NavSatFix, ROOT + '/mission/goal', 10)
        self.nav_pub = self.create_publisher(String, ROOT + '/navigation/status', 10)
        self.bridge_pub = self.create_publisher(BridgeState, ROOT + '/go2/bridge_state', control_qos)
        self.create_subscription(String, ROOT + '/mission/status', self.on_mission, 10)
        self.create_timer(0.1, self.tick)
        self.get_logger().info('Synthetic video test inputs: /vision_test only; no robot connection')

    def on_mission(self, msg):
        try:
            data = json.loads(msg.data)
        except (TypeError, ValueError):
            return
        if isinstance(data, dict):
            self.manager_state = data.get('state', 'IDLE')
            self.goal_accepted |= data.get('goal') == self.goal

    def tick(self):
        now = time.monotonic()
        dt, self.last_tick = max(0.0, now - self.last_tick), now
        if self.manager_state == 'NAVIGATING_DRY_RUN':
            self.distance = max(0.0, self.distance - self.speed * dt)
        if (not self.goal_accepted and self.goal_pub.get_subscription_count() > 0
                and (self.last_goal_sent is None or now - self.last_goal_sent >= 1.0)):
            goal = NavSatFix()
            goal.latitude, goal.longitude = self.goal
            self.goal_pub.publish(goal)
            self.last_goal_sent = now
        self.gps_updates += 1
        nav = dict(
            simulated=True, goal=self.goal, gps_ok=True, satellites_used=12,
            imu_source='external_icm20948', external_imu_ok=True, imu_ok=True,
            gyro_calibrated=True, heading_ok=True, unitree_imu_ok=True,
            go2_telemetry_ok=True, go2_error_code=0, external_mag_valid=True,
            heading_source='video_test_synthetic', heading_true_deg=0.0,
            bearing_true_deg=0.0, heading_error_deg=0.0, distance_m=self.distance,
            gps_updates=self.gps_updates, within_goal_tolerance=self.distance <= 3.5,
            ready_for_dry_run=now - self.started >= self.ready_delay)
        self.nav_pub.publish(String(data=json.dumps(nav, allow_nan=False)))
        bridge = BridgeState()
        bridge.header.stamp = self.get_clock().now().to_msg()
        bridge.state, bridge.reason = 'VIDEO_TEST', 'synthetic_state_no_robot'
        bridge.bridge_epoch = 'video-test-no-motion'
        bridge.connected, bridge.robot_ready = True, True
        bridge.motion_enabled = False
        bridge.transport_error = ''
        self.bridge_pub.publish(bridge)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = VideoTestInputs()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

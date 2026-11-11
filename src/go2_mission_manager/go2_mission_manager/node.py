"""Mission supervisor with automatic preflight and bounded recovery.

The waypoint message is the mission request. The manager waits for fresh,
consistent GPS, external IMU/heading, navigation and bridge state before it
grants motion. Transient sensor/transport failures enter a zero-velocity hold
and resume after readiness returns. A manual bridge stop is never cleared
unless a new waypoint or explicit /mission/start request arrives.
"""
import json
import math
import signal
import time
import uuid

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from go2_interfaces.msg import BridgeState, MotionPermit, MotionCommand
from .speed_ramp import SpeedRamp
from .logic import MissionLogic
from .identification import IdentificationGate


TRANSIENT_BRIDGE_STOPS = {
    'connection_lost', 'robot_not_ready', 'permit_expired', 'command_expired',
}
MOTION_STATES = ('NAVIGATING_ACTIVE', 'GPS_GRACE_HOLD', 'ARRIVED_WAIT_IDENTIFICATION')


def finite_number(value):
    return (float(value) if isinstance(value, (int, float))
            and not isinstance(value, bool) and math.isfinite(value) else None)


def clear_error_code(value):
    """Treat unknown/non-zero sport error codes as blocking, recursively."""
    if value is None or value == '' or value == '0' or value == 0:
        return True
    if isinstance(value, bool):
        return not value
    if isinstance(value, (int, float)):
        return math.isfinite(value) and value == 0
    if isinstance(value, dict):
        return all(clear_error_code(v) for v in value.values())
    if isinstance(value, (list, tuple)):
        return all(clear_error_code(v) for v in value)
    return False


class MissionManager(Node):
    def __init__(self):
        super().__init__('go2_mission_manager')
        defaults = [
            ('enable_motion', False),
            ('status_max_age_sec', 1.0),
            ('arrival_radius_m', 3.5),
            ('arrival_samples', 2),
            ('max_vx', 2.0),
            ('cruise_speed_mps', 0.0),
            ('max_linear_accel_mps2', 0.3),
            ('max_linear_decel_mps2', 0.5),
            ('min_vx', 0.08),
            ('max_wz', 0.25),
            ('max_wz_walking', 0.18),
            ('min_wz', 0.10),
            ('kp_yaw', 0.65),
            ('yaw_cmd_sign', 1.0),
            ('turn_in_place_deg', 22.0),
            ('align_exit_deg', 8.0),
            ('yaw_deadband_deg', 3.5),
            ('slow_down_radius_m', 15.0),
            ('approach_speed_mps', 0.5),
            ('lock_straight_line', False),
            ('auto_start_on_goal', True),
            ('minimum_satellites', 7),
            ('sport_state_code_is_fault', True),
            ('readiness_stable_sec', 1.5),
            ('recovery_retry_sec', 3.0),
            ('max_bridge_recovery_attempts', 3),
            ('identification_initial_radius_m', 50.0),
            ('identification_early_rate_m_per_min', 50.0),
            ('identification_late_rate_m_per_min', 40.0),
            ('identification_breakpoint_sec', 300.0),
            ('identification_result_max_age_sec', 2.0),
        ]
        for name, value in defaults:
            self.declare_parameter(name, value)
        get = lambda name: self.get_parameter(name).value

        self.enable_motion = bool(get('enable_motion'))
        self.status_max_age = float(get('status_max_age_sec'))
        self.auto_start_on_goal = bool(get('auto_start_on_goal'))
        self.min_satellites = int(get('minimum_satellites'))
        self.sport_state_code_is_fault = bool(get('sport_state_code_is_fault'))
        self.readiness_stable_sec = float(get('readiness_stable_sec'))
        self.recovery_retry_sec = float(get('recovery_retry_sec'))
        self.max_recovery_attempts = int(get('max_bridge_recovery_attempts'))
        if (not math.isfinite(self.status_max_age) or self.status_max_age <= 0
                or not math.isfinite(self.readiness_stable_sec) or self.readiness_stable_sec < 0
                or not math.isfinite(self.recovery_retry_sec) or self.recovery_retry_sec <= 0
                or self.min_satellites < 1 or self.max_recovery_attempts < 0):
            raise ValueError('Invalid mission readiness/recovery parameters')

        self.cfg = {
            key: value for key, value in {
                'arrival_radius_m': float(get('arrival_radius_m')),
                'max_vx': float(get('max_vx')),
                'cruise_speed_mps': float(get('cruise_speed_mps')),
                'min_vx': float(get('min_vx')),
                'max_wz': float(get('max_wz')),
                'max_wz_walking': float(get('max_wz_walking')),
                'min_wz': float(get('min_wz')),
                'kp_yaw': float(get('kp_yaw')),
                'yaw_cmd_sign': float(get('yaw_cmd_sign')),
                'turn_in_place_deg': float(get('turn_in_place_deg')),
                'align_exit_deg': float(get('align_exit_deg')),
                'yaw_deadband_deg': float(get('yaw_deadband_deg')),
                'slow_down_radius_m': float(get('slow_down_radius_m')),
                'approach_speed_mps': float(get('approach_speed_mps')),
                'lock_straight_line': bool(get('lock_straight_line')),
            }.items()
        }
        approach = self.cfg['approach_speed_mps']
        if (not math.isfinite(approach)
                or not self.cfg['min_vx'] <= approach <= self.cfg['max_vx']):
            raise ValueError('approach_speed_mps must be between min_vx and max_vx')
        if self.cfg['slow_down_radius_m'] <= self.cfg['arrival_radius_m']:
            raise ValueError('slow_down_radius_m must exceed arrival_radius_m')
        cruise = self.cfg['cruise_speed_mps']
        if (not math.isfinite(cruise)
                or not (cruise == 0.0 or self.cfg['min_vx'] <= cruise <= self.cfg['max_vx'])):
            raise ValueError('cruise_speed_mps must be zero or between min_vx and max_vx')
        self.speed_ramp = SpeedRamp(float(get('max_linear_accel_mps2')),
                                    float(get('max_linear_decel_mps2')))
        self.logic = MissionLogic(arrival_samples=int(get('arrival_samples')))
        self.identification = IdentificationGate(
            initial_radius=float(get('identification_initial_radius_m')),
            early_rate=float(get('identification_early_rate_m_per_min')),
            late_rate=float(get('identification_late_rate_m_per_min')),
            breakpoint_sec=float(get('identification_breakpoint_sec')),
            result_max_age=float(get('identification_result_max_age_sec')))
        self.detector_status = None
        self.detector_status_received = None
        self.wolf_detection = None
        self.nav = None
        self.nav_received = None
        self.bridge = None
        self.bridge_received = None
        self.run_id = ''
        self.epoch = ''
        self.permit_seq = 0
        self.cmd_seq = 0
        self.last_cmd_vel = (0.0, 0.0, 0.0)
        self._goal_rearm_authorized = False
        self._reset_future = None
        self._reset_pending_epoch = False
        self._reset_old_epoch = ''
        self._last_reset_attempt = 0.0
        self._recovery_attempts = 0
        self._preflight_good_since = None
        self._recovery_action = 'idle'
        self._last_rejected_messages = None
        self._last_goal_echo = 0.0

        control_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(NavSatFix, '/mission/goal', self.on_goal, 10)
        self.create_subscription(String, '/navigation/status', self.on_nav, 10)
        self.create_subscription(BridgeState, '/go2/bridge_state', self.on_bridge, control_qos)
        self.permit_pub = self.create_publisher(MotionPermit, '/go2/control_permit', control_qos)
        self.cmd_pub = self.create_publisher(MotionCommand, '/go2/cmd_vel', control_qos)
        self.status_pub = self.create_publisher(String, '/mission/status', 10)
        self.ident_pub = self.create_publisher(Bool, '/identification/enable', 10)
        self.ident_control_pub = self.create_publisher(String, '/identification/control', control_qos)
        self.create_subscription(String, '/identification/result', self.on_identification, control_qos)
        self.create_subscription(String, '/identification/status', self.on_detector_status, control_qos)
        flag_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.wolf_flag_pub = self.create_publisher(Bool, '/mission/wolf_detected', flag_qos)
        self.wolf_flag_pub.publish(Bool(data=False))
        self.goal_echo_pub = self.create_publisher(NavSatFix, '/mission/goal', 10)
        self.stop_client = self.create_client(Trigger, '/go2/stop')
        self.reset_client = self.create_client(Trigger, '/go2/reset_stop')
        self.create_service(Trigger, '/mission/start', self.on_start)
        self.create_service(Trigger, '/mission/pause', self.on_pause)
        self.create_service(Trigger, '/mission/cancel', self.on_cancel)
        self.create_timer(0.05, self.tick)

        mode = 'ACTIVE MOTION ENABLED' if self.enable_motion else 'DRY RUN ONLY'
        self.get_logger().info(
            f'Mission Manager v0.4.0 started ({mode}; auto_start_on_goal={self.auto_start_on_goal}).')

    def _republish_goal_if_needed(self):
        now = time.monotonic()
        if (self.logic.goal and now - self._last_goal_echo >= 1.0
                and (not self.nav or self.nav.get('goal') != self.logic.goal)):
            msg = NavSatFix()
            msg.latitude, msg.longitude = self.logic.goal
            self.goal_echo_pub.publish(msg)
            self._last_goal_echo = now

    def on_goal(self, msg):
        lat, lon = float(msg.latitude), float(msg.longitude)
        if not (math.isfinite(lat) and math.isfinite(lon)
                and -90 <= lat <= 90 and -180 <= lon <= 180):
            self.get_logger().warning('Rejected invalid waypoint coordinates')
            return
        goal = [lat, lon]
        if self.logic.goal == goal:
            return  # includes this node's own goal echo
        self.logic.load_goal(goal)
        self.identification.new_goal(time.monotonic())
        self.wolf_detection = None
        self._publish_identification_control()
        self._goal_rearm_authorized = True  # a fresh waypoint is an explicit new mission request
        self._recovery_attempts = 0
        self._preflight_good_since = None
        self._recovery_action = 'waypoint_received_preflight_pending'
        if self.auto_start_on_goal:
            self.logic.start()
        self.get_logger().info(
            f'Waypoint accepted: lat={lat:.7f}, lon={lon:.7f}; automatic preflight queued')

    def on_nav(self, msg):
        try:
            data = json.loads(msg.data)
            if isinstance(data, dict):
                self.nav = data
                self.nav_received = time.monotonic()
        except (ValueError, TypeError):
            self.nav = {'parse_error': True}
            self.nav_received = time.monotonic()

    def _publish_identification_control(self):
        self.ident_control_pub.publish(String(data=json.dumps(
            self.identification.control(time.time()), allow_nan=False)))
        self.ident_pub.publish(Bool(data=self.identification.enabled))
        self.wolf_flag_pub.publish(Bool(data=self.logic.wolf_detected))

    def on_detector_status(self, msg):
        try:
            data = json.loads(msg.data)
            json.dumps(data, allow_nan=False)
        except (ValueError, TypeError):
            return
        if isinstance(data, dict):
            self.detector_status = data
            self.detector_status_received = time.monotonic()

    def on_identification(self, msg):
        try:
            result = json.loads(msg.data)
            json.dumps(result, allow_nan=False)
        except (ValueError, TypeError):
            return
        if not self.identification.accept_result(result, time.time()):
            return
        if result['detected'] and self.logic.confirm_wolf():
            self.wolf_detection = result
            self._goal_rearm_authorized = False
            self.identification.deactivate()
            self._revoke_and_stop()
            self._recovery_action = 'wolf_confirmed_motion_stopped'
            self._publish_identification_control()
            self.get_logger().warning('Wolf confirmed: motion stopped, /mission/wolf_detected=true')

    def on_bridge(self, msg):
        previous_epoch = self.bridge.bridge_epoch if self.bridge else ''
        self.bridge = msg
        self.bridge_received = time.monotonic()
        if (self._reset_pending_epoch and not msg.stop_requested
                and msg.bridge_epoch != self._reset_old_epoch):
            self._reset_pending_epoch = False
            self._recovery_action = 'bridge_rearmed_preflight_rechecking'
            self.run_id = ''
            self.epoch = ''
            self.permit_seq = self.cmd_seq = 0
        elif previous_epoch and msg.bridge_epoch != previous_epoch:
            self.run_id = ''
            self.epoch = ''
            self.permit_seq = self.cmd_seq = 0

    def _revoke_and_stop(self):
        if self.run_id and self.epoch:
            # Submit zero before revoking the lease; also request the bridge stop latch below.
            self.cmd_seq += 1
            command = MotionCommand()
            command.header.stamp = self.get_clock().now().to_msg()
            command.header.frame_id = 'base_link'
            command.bridge_epoch, command.run_id = self.epoch, self.run_id
            command.sequence = self.cmd_seq
            command.valid_for.nanosec = 180_000_000
            self.cmd_pub.publish(command)
            self.permit_seq += 1
            permit = MotionPermit()
            permit.header.stamp = self.get_clock().now().to_msg()
            permit.header.frame_id = 'base_link'
            permit.bridge_epoch = self.epoch
            permit.run_id = self.run_id
            permit.sequence = self.permit_seq
            permit.enabled = False
            permit.valid_for.nanosec = 400_000_000
            self.permit_pub.publish(permit)
        self.run_id = ''
        self.epoch = ''
        self.permit_seq = self.cmd_seq = 0
        self.speed_ramp.update(0.0, time.monotonic())
        self.last_cmd_vel = (0.0, 0.0, 0.0)
        if self.enable_motion and self.stop_client.service_is_ready():
            self.stop_client.call_async(Trigger.Request())

    def on_start(self, request, response):
        if self.logic.wolf_detected:
            response.success = False
            response.message = 'Wolf confirmed; cancel or submit a new waypoint to leave this phase'
            return response
        if self.logic.goal is None:
            response.success, response.message = False, 'No waypoint loaded'
            return response
        self._goal_rearm_authorized = True  # explicit operator start may clear an old stop latch
        if self.logic.state in self.logic.REQUESTED_STATES:
            response.success = True
            response.message = 'Mission already queued; automatic preflight/recovery continues'
            return response
        response.success, response.message = self.logic.start()
        if response.success:
            self._recovery_attempts = 0
            self._recovery_action = 'manual_start_preflight_pending'
        return response

    def on_pause(self, request, response):
        self._revoke_and_stop()
        self.logic.pause('user_paused_stop_latched')
        self.identification.deactivate()
        self._publish_identification_control()
        self._goal_rearm_authorized = False
        self._recovery_action = 'paused_by_operator'
        response.success, response.message = True, 'Mission paused; stop latch retained until explicit restart/new waypoint'
        return response

    def on_cancel(self, request, response):
        self._revoke_and_stop()
        self.logic.cancel()
        self.identification.clear()
        self.wolf_detection = None
        self._publish_identification_control()
        self._goal_rearm_authorized = False
        self._recovery_action = 'mission_cancelled'
        response.success, response.message = True, 'Mission cancelled and motion inhibited'
        return response

    def _bridge_recovery_allowed(self):
        if self.logic.state not in self.logic.REQUESTED_STATES:
            return False
        if self._goal_rearm_authorized:
            return True
        return bool(self.logic.ever_active and self.bridge
                    and self.bridge.reason in TRANSIENT_BRIDGE_STOPS)

    def _try_reset_bridge(self, now, sensor_ready, bridge_fresh, bridge_connected):
        if not bridge_fresh or not self.bridge or not self.bridge.stop_requested:
            return
        if self._reset_pending_epoch or self._reset_future is not None:
            return
        if not self._bridge_recovery_allowed():
            self._recovery_action = 'manual_stop_latched_waiting_for_new_waypoint_or_start'
            return
        if (not sensor_ready or not bridge_connected or not self.bridge.robot_ready
                or not self.bridge.motion_enabled or self.bridge.transport_error):
            self._recovery_action = 'waiting_for_safe_readiness_before_bridge_reset'
            return
        if self._preflight_good_since is None:
            self._preflight_good_since = now
            self._recovery_action = 'stabilizing_before_bridge_reset'
            return
        if now - self._preflight_good_since < self.readiness_stable_sec:
            self._recovery_action = 'stabilizing_before_bridge_reset'
            return
        if self._recovery_attempts >= self.max_recovery_attempts:
            self._recovery_action = 'bridge_reset_retry_limit_reached'
            return
        if now - self._last_reset_attempt < self.recovery_retry_sec:
            return
        if not self.reset_client.service_is_ready():
            self._recovery_action = 'waiting_for_reset_stop_service'
            return
        self._last_reset_attempt = now
        self._recovery_attempts += 1
        self._reset_old_epoch = self.bridge.bridge_epoch
        self._recovery_action = f'resetting_bridge_stop_latch_attempt_{self._recovery_attempts}'
        self._reset_future = self.reset_client.call_async(Trigger.Request())
        self._reset_future.add_done_callback(self._on_reset_done)

    def _on_reset_done(self, future):
        self._reset_future = None
        try:
            result = future.result()
            if result.success:
                self._goal_rearm_authorized = False
                self._reset_pending_epoch = True
                self._recovery_action = 'bridge_reset_accepted_waiting_for_new_epoch'
            else:
                self._recovery_action = f'bridge_reset_rejected:{result.message}'
        except Exception as exc:  # ROS service can disappear during reconnect
            self._recovery_action = f'bridge_reset_failed:{type(exc).__name__}'

    def _preflight(self, now):
        nav_fresh = (self.nav_received is not None
                     and 0 <= now - self.nav_received <= self.status_max_age)
        bridge_fresh = (self.bridge_received is not None
                        and 0 <= now - self.bridge_received <= self.status_max_age)
        n = self.nav if nav_fresh and isinstance(self.nav, dict) else {}
        b = self.bridge if bridge_fresh else None
        goal_match = bool(self.logic.goal is not None and n.get('goal') == self.logic.goal)
        gps_ok = n.get('gps_ok') is True
        satellites = n.get('satellites_used')
        satellites_ok = (isinstance(satellites, int) and not isinstance(satellites, bool)
                         and satellites >= self.min_satellites)
        external_imu_ready = bool(
            n.get('imu_source') == 'external_icm20948'
            and n.get('external_imu_ok') is True
            and n.get('imu_ok') is True
            and n.get('gyro_calibrated') is True)
        heading_ready = n.get('heading_ok') is True
        unitree_imu_ready = n.get('unitree_imu_ok') is True
        telemetry_ready = n.get('go2_telemetry_ok') is True
        # Go2 Pro WebRTC can expose state/API values in this raw field.
        # Interpret it as a fault only when the selected profile says so.
        robot_fault_clear = (not self.sport_state_code_is_fault
                             or clear_error_code(n.get('go2_error_code')))
        dist = finite_number(n.get('distance_m'))
        heading_error = finite_number(n.get('heading_error_deg'))
        nav_ready = bool(
            n.get('ready_for_dry_run') is True and goal_match and gps_ok and satellites_ok
            and external_imu_ready and heading_ready and unitree_imu_ready
            and telemetry_ready and robot_fault_clear and dist is not None and dist >= 0
            and heading_error is not None)
        bridge_connected = bool(b and b.connected)
        bridge_ready = bool(
            b and b.connected and b.robot_ready
            and (b.motion_enabled or not self.enable_motion)
            and not b.stop_requested and not b.transport_error)
        errors = []
        warnings = []
        if not nav_fresh:
            errors.append('navigation_status_missing_or_stale')
        if nav_fresh and self.logic.goal is not None and not goal_match:
            errors.append('navigation_has_not_received_current_waypoint')
        if nav_fresh and not gps_ok:
            errors.append('gps_fix_or_quality_invalid')
        if nav_fresh and not satellites_ok:
            errors.append(f'insufficient_satellites:{satellites!r}')
        if nav_fresh and not external_imu_ready:
            errors.append('external_imu_or_gyro_calibration_not_ready')
        if nav_fresh and not heading_ready:
            errors.append('heading_invalid')
        if nav_fresh and not unitree_imu_ready:
            errors.append('unitree_relative_imu_not_ready')
        if nav_fresh and not telemetry_ready:
            errors.append('go2_telemetry_stale')
        if nav_fresh and goal_match and (dist is None or dist < 0):
            errors.append('distance_invalid')
        if nav_fresh and goal_match and heading_ready and heading_error is None:
            errors.append('heading_error_invalid')
        if nav_fresh and not robot_fault_clear:
            errors.append(f'go2_fault:{n.get("go2_error_code")!r}')
        if not bridge_fresh:
            errors.append('bridge_state_missing_or_stale')
        elif b:
            if not b.connected:
                errors.append('bridge_disconnected')
            if not b.robot_ready:
                errors.append('robot_not_ready_for_motion')
            if self.enable_motion and not b.motion_enabled:
                errors.append('bridge_motion_disabled')
            if b.transport_error:
                errors.append(f'bridge_transport_error:{b.transport_error}')
            if b.stop_requested:
                errors.append(f'bridge_stop_latched:{b.reason}')
        if not self.enable_motion:
            warnings.append('dry_run_motion_not_enabled')
        if nav_fresh and n.get('heading_source') == 'unitree_imu_magnetic_hold':
            warnings.append('heading_using_bounded_unitree_imu_magnetic_hold')
        if nav_fresh and not n.get('external_mag_valid', True):
            warnings.append('external_magnetometer_temporarily_invalid')
        if nav_fresh and n.get('magnetic_reason'):
            magnetic_reason = str(n.get('magnetic_reason'))
            if magnetic_reason not in ('magnetic_correction', 'magnetic_north_anchored'):
                warnings.append(f'magnetic_status:{magnetic_reason}')
        if b and b.rejected_messages:
            if (self._last_rejected_messages is not None
                    and b.rejected_messages > self._last_rejected_messages):
                warnings.append('bridge_rejected_motion_messages_increased')
            self._last_rejected_messages = b.rejected_messages
        reset_sensor_ready = bool(
            nav_ready and b and b.connected and b.robot_ready and b.motion_enabled
            and not b.transport_error)
        return dict(
            nav=n, bridge=b, nav_fresh=nav_fresh, bridge_fresh=bridge_fresh,
            goal_match=goal_match, gps_ok=gps_ok, external_imu_ready=external_imu_ready,
            heading_ready=heading_ready, nav_ready=nav_ready,
            bridge_connected=bridge_connected, bridge_ready=bridge_ready,
            distance=dist if goal_match else None,
            heading_error=heading_error if goal_match else None, errors=errors,
            warnings=warnings, reset_sensor_ready=reset_sensor_ready)

    def _publish_motion(self, state, checks, now):
        b = checks['bridge']
        if not (self.enable_motion and checks['bridge_ready'] and b):
            self.last_cmd_vel = (0.0, 0.0, 0.0)
            return False, False
        if self.run_id and self.epoch != b.bridge_epoch:
            self.run_id = ''
            self.epoch = ''
            self.permit_seq = self.cmd_seq = 0
        if not self.run_id:
            if state not in MOTION_STATES:
                self.last_cmd_vel = (0.0, 0.0, 0.0)
                return False, False
            self.epoch = b.bridge_epoch
            self.run_id = str(uuid.uuid4())
            self.permit_seq = self.cmd_seq = 0

        self.permit_seq += 1
        permit = MotionPermit()
        permit.header.stamp = self.get_clock().now().to_msg()
        permit.header.frame_id = 'base_link'
        permit.bridge_epoch = self.epoch
        permit.run_id = self.run_id
        permit.sequence = self.permit_seq
        permit.enabled = True
        permit.valid_for.nanosec = 400_000_000
        self.permit_pub.publish(permit)

        if state == 'NAVIGATING_ACTIVE':
            requested = self.logic.compute_velocity(
                checks['distance'], checks['heading_error'], self.cfg)
            vx = self.speed_ramp.update(requested[0], now)
            velocity = (vx, requested[1], requested[2])
        else:
            self.speed_ramp.update(0.0, now)
            velocity = (0.0, 0.0, 0.0)
        self.last_cmd_vel = velocity
        self.cmd_seq += 1
        command = MotionCommand()
        command.header.stamp = self.get_clock().now().to_msg()
        command.header.frame_id = 'base_link'
        command.bridge_epoch = self.epoch
        command.run_id = self.run_id
        command.sequence = self.cmd_seq
        command.valid_for.nanosec = 180_000_000
        command.twist.linear.x, command.twist.linear.y, command.twist.angular.z = velocity
        self.cmd_pub.publish(command)
        return True, True

    def tick(self):
        now = time.monotonic()
        checks = self._preflight(now)
        b = checks['bridge']
        if b and not b.stop_requested:
            # A fresh waypoint only authorizes clearing a stop latch that is
            # already present. It must not authorize a later manual stop.
            self._goal_rearm_authorized = False
            self._preflight_good_since = None
        reset_authorized = bool(
            b and b.stop_requested and self._bridge_recovery_allowed()
            and checks['reset_sensor_ready'])
        if reset_authorized:
            if self._preflight_good_since is None:
                self._preflight_good_since = now
        elif not checks['reset_sensor_ready']:
            self._preflight_good_since = None
        self._try_reset_bridge(now, checks['reset_sensor_ready'],
                               checks['bridge_fresh'], checks['bridge_connected'])

        bridge_ready = checks['bridge_ready']
        if b and b.stop_requested and self._reset_pending_epoch:
            bridge_ready = False
        elif b and b.stop_requested and reset_authorized:
            bridge_ready = False
        if self._reset_pending_epoch:
            checks['bridge_ready'] = False

        within = bool(
            checks['gps_ok'] and checks['distance'] is not None
            and checks['distance'] <= self.cfg['arrival_radius_m']
            and checks['nav'].get('within_goal_tolerance') is True)
        previous_state = self.logic.state
        self.logic.update(
            gps_ok=checks['gps_ok'], nav_ready=checks['nav_ready'],
            bridge_connected=checks['bridge_connected'], bridge_ready=bridge_ready,
            within_goal=within, gps_updates=checks['nav'].get('gps_updates'),
            enable_motion=self.enable_motion, now=now,
            readiness_stable_sec=self.readiness_stable_sec,
            distance_m=checks['distance'])

        # A hard bridge stop is held until an explicit fresh waypoint/start, or
        # until a known transient transport fault has safely recovered.
        if b and b.stop_requested:
            self.run_id = ''
            self.epoch = ''
            self.permit_seq = self.cmd_seq = 0
            self.speed_ramp.update(0.0, now)
            self.last_cmd_vel = (0.0, 0.0, 0.0)
        elif (previous_state in MOTION_STATES
              and self.logic.state not in MOTION_STATES
              and self.logic.state != 'WAIT_LOCALIZATION'):
            # Pause/cancel is an explicit stop; normal arrival instead retains
            # an enabled zero-velocity hold so the next waypoint needs no reset.
            if self.logic.state in ('PAUSED', 'IDLE'):
                self._revoke_and_stop()

        # If a waypoint replaces an active one, keep issuing zero on the existing
        # lease while the navigation node consumes and confirms the new target.
        motion_state = self.logic.state in MOTION_STATES
        replacing_goal_hold = self.logic.state == 'WAIT_LOCALIZATION' and bool(self.run_id)
        if motion_state or replacing_goal_hold:
            permit_sent, cmd_sent = self._publish_motion(self.logic.state, checks, now)
        else:
            permit_sent = cmd_sent = False
            if self.logic.state != 'WAIT_LOCALIZATION':
                self.last_cmd_vel = (0.0, 0.0, 0.0)
                self.speed_ramp.update(0.0, now)

        if (self.logic.goal and self.logic.state != 'IDLE'
                and (not self.nav or self.nav.get('goal') != self.logic.goal)):
            self._republish_goal_if_needed()

        if self.logic.state == 'IDLE':
            self._recovery_action = 'waiting_for_waypoint'
        elif self.logic.state == 'NAVIGATING_ACTIVE':
            self._goal_rearm_authorized = False
            self._recovery_action = 'mission_running'
        elif self.logic.state == 'ARRIVED_WAIT_IDENTIFICATION':
            self._goal_rearm_authorized = False
            self._recovery_action = 'arrived_zero_velocity_hold'
        elif self.logic.state == 'WOLF_DETECTED':
            self._goal_rearm_authorized = False
            self._recovery_action = 'wolf_confirmed_motion_stopped'
        elif checks['errors'] and not (b and b.stop_requested and reset_authorized):
            if self._recovery_action in ('idle', 'mission_running'):
                if not checks['nav_fresh'] or not checks['bridge_fresh']:
                    self._recovery_action = 'waiting_for_launch_respawn_or_fresh_status'
                else:
                    self._recovery_action = 'waiting_for_preflight_conditions'
        elif self.logic.state == 'WAIT_LOCALIZATION' and not checks['errors']:
            self._recovery_action = 'preflight_stabilizing'

        ident_enable = self.identification.update(
            self.logic.state, checks['distance'],
            checks['nav_fresh'] and checks['goal_match'] and checks['gps_ok'],
            now, time.time())
        self._publish_identification_control()
        n = checks['nav']
        reported_distance = (self.logic.arrival_distance_m if self.logic.state == 'ARRIVED_WAIT_IDENTIFICATION'
                             and self.logic.arrival_distance_m is not None
                             else checks['distance'])
        status = {
            'state': self.logic.state,
            'reason': self.logic.reason,
            'goal': self.logic.goal,
            'distance_m': reported_distance,
            'live_distance_m': checks['distance'],
            'arrival_distance_m': self.logic.arrival_distance_m,
            'arrival_radius_m': self.cfg['arrival_radius_m'],
            'arrival_samples': self.logic.arrival_samples,
            'bearing_true_deg': (finite_number(n.get('bearing_true_deg'))
                                 if checks['goal_match'] else None),
            'heading_true_deg': finite_number(n.get('heading_true_deg')),
            'heading_error_deg': checks['heading_error'],
            'heading_source': n.get('heading_source'),
            'go2_sport_state_code': n.get('go2_error_code'),
            'sport_state_code_interpretation': (
                'fault_code' if self.sport_state_code_is_fault else 'informational'),
            'magnetic_mode': n.get('magnetic_mode'),
            'magnetic_reason': n.get('magnetic_reason'),
            'aligning_in_place': self.logic.aligning_in_place,
            'cmd_vel': list(self.last_cmd_vel),
            'satellites_used': n.get('satellites_used'),
            'gps_ok': checks['gps_ok'],
            'external_imu_ready': checks['external_imu_ready'],
            'heading_ready': checks['heading_ready'],
            'navigation_ready': checks['nav_ready'],
            'bridge_connected': checks['bridge_connected'],
            'bridge_ready': checks['bridge_ready'],
            'bridge_stop_requested': bool(b and b.stop_requested),
            'bridge_stop_reason': b.reason if b else None,
            'bridge_stop_confirmed': bool(b and b.stop_confirmed),
            'preflight_errors': checks['errors'],
            'preflight_warnings': checks['warnings'],
            'recovery_action': self._recovery_action,
            'recovery_attempts': self._recovery_attempts,
            'identification_enable': ident_enable,
            'mission_id': self.identification.mission_id,
            'waypoint_age_sec': self.identification.age(now),
            'identification_radius_m': self.identification.radius(now),
            'identification_session_id': self.identification.session_id,
            'wolf_detected': self.logic.wolf_detected,
            'wolf_detection': self.wolf_detection,
            'detector_status': self.detector_status,
            'detector_status_fresh': bool(self.detector_status_received is not None
                                          and now - self.detector_status_received <= 2.0),
            'motion_permit_sent': permit_sent,
            'motion_command_sent': cmd_sent,
            'mode': 'DIRECT_WAYPOINT_ACTIVE' if self.enable_motion else 'DRY_RUN_ONLY',
            'stamp_unix': time.time(),
        }
        self.status_pub.publish(String(data=json.dumps(status, allow_nan=False)))


def main(args=None):
    # Keep ROS valid until the stop/revoke messages have been submitted.
    # The default rclpy signal handler invalidates the context before finally.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, signal.default_int_handler)
    signal.signal(signal.SIGTERM, signal.default_int_handler)
    node = MissionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._revoke_and_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

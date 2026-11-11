"""ROS 2 facade. Motion messages carry the current bridge epoch and run ID."""
import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import String
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image, Imu
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from go2_interfaces.msg import MotionPermit, MotionCommand, BridgeState

from .core import MotionGate, Limits, remaining_ttl
from .worker import TransportWorker


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class Go2Bridge(Node):
    def __init__(self):
        super().__init__('go2_bridge')
        defaults = dict(
            backend='webrtc', connection_method='LocalSTA', robot_ip='',
            motion_enabled=False, ready_modes='', video_enabled=True,
            motion_transport='sport', joystick_max_forward=0.42,
            joystick_max_lateral=0.20, joystick_max_yaw=0.20,
            video_rate=5.0, camera_frame='go2_camera_optical_frame',
            command_frame='base_link', command_rate=20.0,
            permit_timeout=0.6, command_timeout=0.3, telemetry_timeout=0.5,
            connect_timeout=25.0, reconnect_delay=3.0, max_buffered_bytes=16384,
            max_vx=2.0, max_vy=2.0, max_wz=0.30, max_clock_skew=0.05)
        for name, value in defaults.items():
            self.declare_parameter(name, value, ParameterDescriptor(read_only=True))
        config = {key: self.get_parameter(key).value for key in defaults}
        if self.get_parameter('use_sim_time').value:
            raise ValueError('This bridge uses wall time for source stamps; use_sim_time must be false')
        if config['backend'] not in ('webrtc', 'mock'):
            raise ValueError('backend must be webrtc or mock; SDK2 is not implemented')
        if config['motion_transport'] not in ('sport', 'virtual_joystick'):
            raise ValueError('motion_transport must be sport or virtual_joystick')
        if not all(math.isfinite(config[k]) and 0 < config[k] <= 1 for k in
                   ('joystick_max_forward', 'joystick_max_lateral', 'joystick_max_yaw')):
            raise ValueError('joystick scales must be finite and between 0 and 1')
        config['ready_modes'] = [int(s.strip()) for s in config['ready_modes'].split(',') if s.strip()]
        for key in ('video_rate', 'command_rate', 'telemetry_timeout', 'connect_timeout',
                    'reconnect_delay', 'max_buffered_bytes', 'max_clock_skew'):
            if not math.isfinite(config[key]) or config[key] <= 0:
                raise ValueError(f'{key} must be finite and positive')
        if not config['command_frame'] or not config['camera_frame']:
            raise ValueError('Frame names must not be empty')
        self.config = config
        self.gate = MotionGate(Limits(config['permit_timeout'], config['command_timeout'],
                                     config['max_vx'], config['max_vy'], config['max_wz']),
                               config['motion_enabled'])
        self.worker = TransportWorker(self.gate, config)
        control_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.VOLATILE)
        sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                                durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(MotionPermit, 'control_permit', self.permit, control_qos)
        self.create_subscription(MotionCommand, 'cmd_vel', self.command, control_qos)
        self.create_service(Trigger, 'stop', self.stop)
        self.create_service(Trigger, 'reset_stop', self.reset_stop)
        self.status_pub = self.create_publisher(BridgeState, 'bridge_state', control_qos)
        self.diagnostics_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.raw_pub = self.create_publisher(String, 'debug/sport_state_json', sensor_qos)
        self.image_pub = self.create_publisher(Image, 'camera/image_raw', sensor_qos)
        self.imu_pub = self.create_publisher(Imu, 'imu/data', sensor_qos)
        self.last_raw_wall = 0
        self.last_report = None
        self.rejected = 0
        self.create_timer(0.1, self.publish_state)
        self.create_timer(1.0/config['video_rate'], self.publish_image)
        self.worker.start()

    def remaining(self, stamp, duration, max_ttl):
        return remaining_ttl(seconds(stamp), seconds(duration),
                             self.get_clock().now().nanoseconds*1e-9,
                             max_ttl, self.config['max_clock_skew'])

    def permit(self, msg):
        # Bound TTL before accounting for transport delay, not afterwards.
        ttl = self.remaining(msg.header.stamp, msg.valid_for, self.config['permit_timeout'])
        with self.worker.lock:
            accepted = ttl is not None and ttl > 0 and self.gate.permit(
                msg.bridge_epoch, msg.run_id, msg.sequence, msg.enabled, ttl, time.monotonic())
        if not accepted:
            self.rejected += 1

    def command(self, msg):
        ttl = self.remaining(msg.header.stamp, msg.valid_for, self.config['command_timeout'])
        t = msg.twist
        extra = (t.linear.z, t.angular.x, t.angular.y)
        supported = msg.header.frame_id == self.config['command_frame'] and all(v == 0.0 for v in extra)
        with self.worker.lock:
            accepted = supported and ttl is not None and ttl > 0 and self.gate.command(
                msg.bridge_epoch, msg.run_id, msg.sequence,
                (t.linear.x, t.linear.y, t.angular.z), ttl, time.monotonic())
        if not accepted:
            self.rejected += 1

    def stop(self, request, response):
        with self.worker.lock:
            self.gate.trip('stop_service')
        response.success = True
        response.message = 'Local motion inhibited; stop queued if transport available. Physical stop NOT confirmed.'
        return response

    def reset_stop(self, request, response):
        with self.worker.lock:
            response.success = self.gate.reset_stop()
            if response.success:
                self.worker.stop_sent = False
                self.worker.stop_count = 0
        response.message = ('Local latch reset; NEW epoch, permit and command required. '
                            'Physical standstill is not verified here.' if response.success
                            else 'Robot/telemetry not ready; reset rejected.')
        return response

    def publish_state(self):
        info = self.worker.snapshot()
        msg = BridgeState()
        msg.header.stamp = self.get_clock().now().to_msg()
        for key in ('state', 'reason', 'bridge_epoch', 'run_id', 'connected',
                    'robot_ready', 'motion_enabled', 'permit_valid', 'stop_requested',
                    'stop_sent', 'stop_confirmed', 'transport_error'):
            setattr(msg, key, info[key])
        msg.telemetry_age = float(info['telemetry_age']) if math.isfinite(info['telemetry_age']) else -1.0
        msg.rejected_messages = self.rejected
        self.status_pub.publish(msg)
        report = (msg.state, msg.reason, msg.transport_error)
        if report != self.last_report:
            self.get_logger().info(' / '.join(report))
            self.last_report = report
        d = DiagnosticStatus(name='go2_bridge', hardware_id='go2', message=msg.reason)
        d.level = DiagnosticStatus.OK if msg.state in ('READY', 'ENABLED', 'READ_ONLY') else DiagnosticStatus.WARN
        d.values = [KeyValue(key=k, value=str(v)) for k, v in info.items()]
        self.diagnostics_pub.publish(DiagnosticArray(header=msg.header, status=[d]))
        with self.worker.lock:
            raw, wall = self.worker.raw_state, self.worker.state_wall
        if raw is not None and wall != self.last_raw_wall:
            self.last_raw_wall = wall
            self.raw_pub.publish(String(data=json.dumps({'received_unix_ns': wall, 'data': raw})))
            if (info['connected'] and
                    0 <= info['telemetry_age'] < self.config['telemetry_timeout']):
                self.publish_unitree_imu(raw, wall)


    def publish_unitree_imu(self, raw, wall):
        """Publish fresh Unitree sport-state IMU. Quaternion input is WXYZ."""
        try:
            state = raw.get('imu_state')
            if not isinstance(state, dict):
                return

            q = state['quaternion']
            gyro = state['gyroscope']
            accel = state['accelerometer']

            if len(q) != 4 or len(gyro) != 3 or len(accel) != 3:
                return

            values = [float(v) for v in (*q, *gyro, *accel)]
            if not all(math.isfinite(v) for v in values):
                return

            qw, qx, qy, qz = values[:4]
            norm = math.sqrt(qw*qw + qx*qx + qy*qy + qz*qz)
            if not 0.8 <= norm <= 1.2:
                return

            msg = Imu()
            msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(
                int(wall), 1_000_000_000
            )
            msg.header.frame_id = 'go2_imu_link'

            # Unitree WXYZ -> ROS XYZW.
            msg.orientation.x = qx / norm
            msg.orientation.y = qy / norm
            msg.orientation.z = qz / norm
            msg.orientation.w = qw / norm

            msg.angular_velocity.x = values[4]
            msg.angular_velocity.y = values[5]
            msg.angular_velocity.z = values[6]

            msg.linear_acceleration.x = values[7]
            msg.linear_acceleration.y = values[8]
            msg.linear_acceleration.z = values[9]

            # Covariances left at zero: unknown, NOT zero uncertainty.
            # Axes, mounting and absolute yaw still require validation.
            self.imu_pub.publish(msg)

        except (KeyError, TypeError, ValueError, OverflowError):
            return

    def publish_image(self):
        with self.worker.lock:
            item, self.worker.frame = self.worker.frame, None
        if item is None or self.image_pub.get_subscription_count() == 0:
            return
        frame, wall = item
        try:
            bgr = frame.to_ndarray(format='bgr24')
            msg = Image()
            msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(wall, 1_000_000_000)
            msg.header.frame_id = self.config['camera_frame']
            msg.height, msg.width = bgr.shape[:2]
            msg.encoding, msg.is_bigendian = 'bgr8', 0
            msg.step = msg.width * 3
            msg.data = bgr.tobytes()
            self.image_pub.publish(msg)
        except Exception as exc:
            self.get_logger().warning(f'Video conversion failed: {type(exc).__name__}')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Go2Bridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            if not node.worker.close():
                node.get_logger().warning('Transport thread still closing; physical stop not confirmed')
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

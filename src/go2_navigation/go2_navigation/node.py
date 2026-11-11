# Navigation: external magnetic north with bounded relative-yaw support over magnetic anomalies.
import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Bool, Float64, String
from .heading_fusion import HeadingFusion
from .geodesy import valid_coordinates, distance_and_bearing, heading_error_deg, gps_quality_ok

class NavigationNode(Node):
    def __init__(self):
        super().__init__('go2_navigation')
        for name, val in [
            ('min_satellites', 7),
            ('gps_max_age_sec', 3.5),
            ('gga_max_age_sec', 3.5),
            ('imu_max_age_sec', 1.5),
            ('telemetry_max_age_sec', 1.5),
            ('arrival_radius_m', 3.5),
            ('unitree_imu_max_age_sec', 0.5),
            ('magnetic_hold_max_sec', 10.0),
            ('magnetic_innovation_limit_deg', 12.0),
            ('magnetic_reacquire_sec', 1.0),
            ('magnetic_correction_rate_dps', 2.0),
            ('unitree_yaw_ccw_positive', True),
        ]:
            self.declare_parameter(name, val)

        self.min_sats = int(self.get_parameter('min_satellites').value)
        self.gps_max_age = float(self.get_parameter('gps_max_age_sec').value)
        self.gga_max_age = float(self.get_parameter('gga_max_age_sec').value)
        self.imu_max_age = float(self.get_parameter('imu_max_age_sec').value)
        self.telemetry_max_age = float(self.get_parameter('telemetry_max_age_sec').value)
        self.arrival_radius = float(self.get_parameter('arrival_radius_m').value)
        self.unitree_imu_max_age = float(self.get_parameter('unitree_imu_max_age_sec').value)
        if not math.isfinite(self.unitree_imu_max_age) or self.unitree_imu_max_age <= 0:
            raise ValueError('Invalid Unitree IMU freshness')
        self.fusion = HeadingFusion(
            float(self.get_parameter('magnetic_hold_max_sec').value),
            float(self.get_parameter('magnetic_innovation_limit_deg').value),
            float(self.get_parameter('magnetic_reacquire_sec').value),
            float(self.get_parameter('magnetic_correction_rate_dps').value),
            bool(self.get_parameter('unitree_yaw_ccw_positive').value))
        self.unitree_yaw_deg = None
        self.unitree_imu_ok = False
        self.external_mag_valid = False
        self.fusion_state = {}
        self.gps = self.gps_quality = self.imu = self.ext_heading = self.goal = None
        self.imu_quality = None
        self.ext_heading_valid = False
        self.telemetry = None
        self.recv = {}
        self.gps_updates = 0

        self.heading_source = 'none'

        self.create_subscription(NavSatFix, '/gps/fix', self.on_fix, qos_profile_sensor_data)
        self.create_subscription(String, '/gps/quality', self.on_quality, qos_profile_sensor_data)
        self.create_subscription(Imu, '/imu/data', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(Imu, '/go2/imu/data', self.on_unitree_imu, qos_profile_sensor_data)
        self.create_subscription(String, '/imu/quality', self.on_imu_quality, qos_profile_sensor_data)
        self.create_subscription(Float64, '/navigation/heading_true_deg', self.on_heading, 10)
        self.create_subscription(Bool, '/navigation/heading_valid', self.on_heading_valid, 10)
        self.create_subscription(String, '/go2/debug/sport_state_json', self.on_sport, qos_profile_sensor_data)
        self.create_subscription(NavSatFix, '/mission/goal', self.on_goal, 10)

        self.pub = self.create_publisher(String, '/navigation/status', 10)
        self.create_timer(0.1, self.tick)
        self.get_logger().info('Navigation started (external magnetic north + bounded Unitree relative yaw on magnetic anomalies).')

    def on_fix(self, m):
        self.gps = m
        self.recv['gps'] = time.monotonic()
        self.gps_updates += 1

    def on_quality(self, m):
        try:
            q = json.loads(m.data)
            if isinstance(q, dict):
                self.gps_quality = q
                self.recv['quality'] = time.monotonic()
        except (ValueError, TypeError):
            pass

    def on_imu(self, m):
        values = [m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z,
                  m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z]
        stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        age = self.get_clock().now().nanoseconds * 1e-9 - stamp
        if not all(math.isfinite(v) for v in values) or not 0 <= age <= self.imu_max_age:
            return
        self.imu = m
        self.recv['imu'] = time.monotonic()

    def on_unitree_imu(self, m):
        q = m.orientation
        values = (q.x, q.y, q.z, q.w)
        stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        age = self.get_clock().now().nanoseconds * 1e-9 - stamp
        if not all(math.isfinite(v) for v in values) or not 0 <= age <= self.unitree_imu_max_age:
            return
        length = math.sqrt(sum(v*v for v in values))
        if not 0.95 <= length <= 1.05:
            return
        x, y, z, w = (v/length for v in values)
        self.unitree_yaw_deg = math.degrees(math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z)))
        self.recv['unitree_imu'] = time.monotonic()

    def on_imu_quality(self, m):
        try:
            q = json.loads(m.data)
            if isinstance(q, dict):
                self.imu_quality = q
                self.recv['imu_quality'] = time.monotonic()
        except (ValueError, TypeError):
            pass

    def on_heading(self, m):
        if math.isfinite(m.data):
            self.ext_heading = float(m.data) % 360.0
            self.recv['heading'] = time.monotonic()

    def on_heading_valid(self, m):
        self.ext_heading_valid = bool(m.data)
        self.recv['heading_valid'] = time.monotonic()

    def on_sport(self, m):
        try:
            payload = json.loads(m.data)
            data = payload.get('data')
            if isinstance(data, dict):
                self.telemetry = data
                self.recv['sport'] = time.monotonic()
        except (ValueError, TypeError):
            pass

    def on_goal(self, m):
        if valid_coordinates(m.latitude, m.longitude):
            self.goal = [float(m.latitude), float(m.longitude)]
            self.get_logger().info(f'Goal loaded: lat={m.latitude:.7f}, lon={m.longitude:.7f}')

    def fresh(self, key, now, threshold):
        return key in self.recv and 0 <= now - self.recv[key] <= threshold

    def compute_fused_heading(self, now, imu_ok):
        iq = self.imu_quality or {}
        heading = iq.get('heading_preview_true_deg')
        stamp = iq.get('stamp_unix')
        source_fresh = (isinstance(stamp, (int, float)) and math.isfinite(stamp)
                        and 0 <= time.time() - stamp <= self.imu_max_age)
        self.external_mag_valid = bool(
            imu_ok and source_fresh and self.fresh('imu_quality', now, self.imu_max_age)
            and iq.get('heading_valid') is True and iq.get('calibration_loaded') is True
            and iq.get('mag_ok') is True and isinstance(heading, (int, float))
            and not isinstance(heading, bool) and math.isfinite(heading) and 0 <= heading < 360)
        self.unitree_imu_ok = bool(
            self.unitree_yaw_deg is not None and self.fresh('unitree_imu', now, self.unitree_imu_max_age)
            and self.fresh('sport', now, self.telemetry_max_age))
        self.fusion_state = self.fusion.update(
            now, self.unitree_yaw_deg, self.unitree_imu_ok,
            heading, self.external_mag_valid, stamp)
        valid = bool(imu_ok and self.fusion_state['valid'])
        self.heading_source = self.fusion_state['source'] if valid else 'none'
        return (self.fusion_state['heading_deg'], True) if valid else (None, False)

    def tick(self):
        now = time.monotonic()
        q = self.gps_quality or {}
        n = q.get('satellites_used')
        gga_age = q.get('gga_age_s')
        if gga_age is None and self.fresh('gps', now, self.gps_max_age):
            gga_age = now - self.recv['gps']

        gps_ok = bool(self.gps is not None and self.fresh('gps', now, self.gps_max_age)
                      and self.fresh('quality', now, self.gga_max_age)
                      and self.gps.status.status >= 0
                      and valid_coordinates(self.gps.latitude, self.gps.longitude)
                      and q.get('fix_valid') is True
                      and gps_quality_ok(q.get('gga_quality'), n, gga_age,
                                         self.min_sats, self.gga_max_age))

        iq = self.imu_quality or {}
        gyro_calibrated = bool(iq.get('gyro_calibrated') is True
                               and iq.get('imu_ok') is True
                               and self.fresh('imu_quality', now, self.imu_max_age))
        sport_ok = bool(self.telemetry is not None and self.fresh('sport', now, self.telemetry_max_age))
        imu_ok = bool(self.imu is not None and self.fresh('imu', now, self.imu_max_age)
                      and iq.get('imu_ok') is True
                      and self.fresh('imu_quality', now, self.imu_max_age))

        fused_heading, heading_ok = self.compute_fused_heading(now, imu_ok)

        dist = bearing = turn = None
        if gps_ok and self.goal is not None:
            dist, bearing = distance_and_bearing(self.gps.latitude, self.gps.longitude, *self.goal)
            if heading_ok and fused_heading is not None:
                turn = heading_error_deg(bearing, fused_heading)

        ready = bool(gps_ok and imu_ok and heading_ok and sport_ok and self.unitree_imu_ok and self.goal)
        out = {
            'mode': 'MAGNETIC_NORTH_WITH_RELATIVE_IMU',
            'gps_ok': gps_ok,
            'satellites_used': n,
            'gga_quality': q.get('gga_quality'),
            'gps_updates': self.gps_updates,
            'heading_ok': heading_ok,
            'heading_source': self.heading_source,
            'heading_true_deg': round(fused_heading, 2) if fused_heading is not None else None,
            'external_mag_heading_deg': round(self.ext_heading, 2) if self.ext_heading is not None else None,
            'external_mag_valid': self.external_mag_valid,
            'external_mag_accepted': self.fusion_state.get('mag_accepted', False),
            'north_reference_source': 'external_icm20948_magnetometer',
            'relative_rotation_source': 'unitree_imu',
            'unitree_imu_ok': self.unitree_imu_ok,
            'unitree_yaw_deg': self.unitree_yaw_deg,
            'magnetic_mode': self.fusion_state.get('mode'),
            'magnetic_reason': self.fusion_state.get('reason'),
            'magnetic_hold_age_sec': self.fusion_state.get('mag_age_sec'),
            'magnetic_innovation_deg': self.fusion_state.get('innovation_deg'),
            'imu_ok': imu_ok,
            'imu_source': 'external_icm20948',
            'external_imu_ok': imu_ok,
            'gyro_calibrated': gyro_calibrated,
            'go2_telemetry_ok': sport_ok,
            'go2_mode': self.telemetry.get('mode') if self.telemetry else None,
            'go2_error_code': self.telemetry.get('error_code') if self.telemetry else None,
            'local_position_raw': self.telemetry.get('position') if self.telemetry else None,
            'local_velocity_raw': self.telemetry.get('velocity') if self.telemetry else None,
            'goal_loaded': self.goal is not None,
            'goal': self.goal,
            'ready_for_dry_run': ready,
            'distance_m': round(dist, 2) if dist is not None else None,
            'bearing_true_deg': round(bearing, 2) if bearing is not None else None,
            'heading_error_deg': round(turn, 2) if turn is not None else None,
            'within_goal_tolerance': bool(ready and dist is not None and dist <= self.arrival_radius),
            'stamp_unix': time.time(),
        }
        self.pub.publish(String(data=json.dumps(out, allow_nan=False)))

def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

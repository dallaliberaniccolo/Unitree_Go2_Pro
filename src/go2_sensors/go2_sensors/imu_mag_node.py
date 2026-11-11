# ICM-20948 (SPI0) + AK09916 magnetometer ROS node.
# FIXED: Gyro calibration never blocks magnetometer reading or compass heading.
import json
import math
import time
from collections import deque
from pathlib import Path
from statistics import mean, pstdev

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Bool, Float64, String

from .gravity_filter import GravityFilter
from .horizontal_calibration import load_horizontal_calibration, calibrated_horizontal_field
from .icm20948_spi import ICM20948SPI
from .math_sensor import (apply_axis_map, calibrated_field, calibration_from_json,
                          compass_2d_deg, norm, tilt_compass_deg)

class ImuMagNode(Node):
    def __init__(self):
        super().__init__('go2_imu_mag_sensor')
        for name, value in [
            ('spi_bus', 0), ('spi_device', 0), ('spi_speed_hz', 100000),
            ('rate_hz', 10.0), ('accel_dlpf_cfg', 6), ('gyro_dlpf_cfg', 6),
            ('gravity_tau_sec', 0.5), ('gravity_accel_smoothing_sec', 0.2), ('gravity_max_coast_sec', 2.0), ('calibration_path', '/sensor_calib/mag_calibrazione.json'),
            ('axes_verified', True), ('declination_configured', True),
            ('declination_deg', 4.21), ('heading_offset_deg', 0.0),
            ('use_2d_horizontal_compass', False),
            ('accel_gyro_axes', [1, 2, 3]), ('mag_axes', [1, -2, -3]),
            ('max_mag_variation_percent', 55.0), ('max_mag_deviation_fraction', 0.45),
            ('max_mag_upper_deviation_fraction', 0.45),
            ('gyro_auto_calibrate', True), ('gyro_calib_seconds', 2.5),
            ('gyro_calib_max_std_dps', 2.5), ('gyro_calib_max_acc_std_g', 0.08),
            ('gyro_calib_max_mean_dps', 10.0),
            ('heading_accel_alpha', 0.15)]:
            self.declare_parameter(name, value)

        get = lambda n: self.get_parameter(n).value
        self.acc_axes = tuple(int(x) for x in get('accel_gyro_axes'))
        self.mag_axes = tuple(int(x) for x in get('mag_axes'))
        apply_axis_map((1., 2., 3.), self.acc_axes)
        apply_axis_map((1., 2., 3.), self.mag_axes)

        self.axes_verified = bool(get('axes_verified'))
        self.declination_configured = bool(get('declination_configured'))
        self.declination = float(get('declination_deg'))
        self.heading_offset = float(get('heading_offset_deg'))
        self.use_2d_compass = bool(get('use_2d_horizontal_compass'))
        self.max_variation = float(get('max_mag_variation_percent'))
        self.max_deviation = float(get('max_mag_deviation_fraction'))
        self.max_upper_deviation = float(get('max_mag_upper_deviation_fraction'))
        if not (math.isfinite(self.max_deviation) and 0 <= self.max_deviation < 1
                and math.isfinite(self.max_upper_deviation) and self.max_upper_deviation >= 0):
            raise ValueError('Invalid magnetic field acceptance range')
        hz = float(get('rate_hz'))
        self.heading_accel_alpha = float(get('heading_accel_alpha'))
        self.gravity_filter = GravityFilter(float(get('gravity_tau_sec')), float(get('gravity_max_coast_sec')), float(get('gravity_accel_smoothing_sec')))
        self.heading_accel_lp = None
        self.heading_sin_lp = None
        self.heading_cos_lp = None

        self.gyro_auto = bool(get('gyro_auto_calibrate'))
        self.gyro_calib_seconds = float(get('gyro_calib_seconds'))
        self.gyro_std_limit = float(get('gyro_calib_max_std_dps'))
        self.acc_std_limit = float(get('gyro_calib_max_acc_std_g'))
        self.gyro_mean_limit = float(get('gyro_calib_max_mean_dps'))
        self.gyro_calibrated = not self.gyro_auto
        self.gyro_bias = (0.0, 0.0, 0.0)
        self.calib_samples = deque(maxlen=500)
        self.calib_last_reason = 'waiting_for_stationary_window'

        self.calibration = None
        self.horizontal_calibration = None
        candidate = Path(str(get('calibration_path')))
        if candidate.is_file():
            try:
                raw_calibration = json.loads(candidate.read_text())
                if raw_calibration.get('kind') == 'horizontal_2d':
                    horizontal = load_horizontal_calibration(raw_calibration)
                    if not self.use_2d_compass or self.mag_axes != horizontal['axes']:
                        raise ValueError('Horizontal calibration requires matching axes and 2D compass mode')
                    self.horizontal_calibration = horizontal
                else:
                    self.calibration = calibration_from_json(raw_calibration)
                self.get_logger().info(f'Calibration loaded from: {candidate}')
            except (ValueError, KeyError, TypeError, OSError) as exc:
                self.get_logger().error(f'Invalid calibration {candidate}: {exc}')
        else:
            self.get_logger().error(f'Calibration missing: {candidate}; heading stays invalid')

        self.sensor = ICM20948SPI(bus=int(get('spi_bus')), device=int(get('spi_device')),
                                  speed_hz=int(get('spi_speed_hz')),
                                  accel_dlpf_cfg=int(get('accel_dlpf_cfg')), gyro_dlpf_cfg=int(get('gyro_dlpf_cfg')))
        self.sensor_recovery_pending = False
        self.sensor_recovery_attempts = 0
        self.sensor_last_error = ''
        self._sensor_recovery_times = deque()
        self._next_sensor_recovery = 0.0
        self.imu_pub = self.create_publisher(Imu, '/imu/data', qos_profile_sensor_data)
        self.mag_pub = self.create_publisher(MagneticField, '/imu/mag', qos_profile_sensor_data)
        self.quality_pub = self.create_publisher(String, '/imu/quality', qos_profile_sensor_data)
        self.heading_pub = self.create_publisher(Float64, '/navigation/heading_true_deg', 10)
        self.valid_pub = self.create_publisher(Bool, '/navigation/heading_valid', 10)
        self.create_timer(1.0 / hz, self.tick)
        self.get_logger().info('ICM20948 node active (non-blocking gyro + 2D/3D compass).')

    def request_sensor_recovery(self, quality, reason):
        """Invalidate calibration/filter state before any reset/recovery attempt."""
        self.sensor_last_error = reason
        self.sensor_recovery_pending = True
        self.gyro_calibrated = False
        self.gyro_bias = (0.0, 0.0, 0.0)
        self.calib_samples.clear()
        self.calib_last_reason = 'sensor_recovery_pending'
        gravity = self.gravity_filter
        self.gravity_filter = GravityFilter(gravity.tau, gravity.max_coast, gravity.accel_tau)
        self.heading_accel_lp = None
        self.heading_sin_lp = None
        self.heading_cos_lp = None
        quality.update(imu_ok=False, mag_ok=False, heading_valid=False,
                       gyro_calibrated=False, gyro_bias_dps=[0.0, 0.0, 0.0],
                       gyro_calibration_reason=self.calib_last_reason,
                       sensor_recovery_pending=True, sensor_last_error=reason,
                       reason=reason)

    def recover_sensor(self, quality):
        """At most three full initializations per minute, separated by two seconds."""
        now = time.monotonic()
        while self._sensor_recovery_times and now - self._sensor_recovery_times[0] >= 60.0:
            self._sensor_recovery_times.popleft()
        if now < self._next_sensor_recovery:
            quality['reason'] = 'sensor_recovery_waiting'
        elif len(self._sensor_recovery_times) >= 3:
            quality['reason'] = 'sensor_recovery_rate_limited'
        else:
            self._sensor_recovery_times.append(now)
            self._next_sensor_recovery = now + 2.0
            self.sensor_recovery_attempts += 1
            try:
                self.sensor.reinitialize()
            except (RuntimeError, OSError, ValueError) as exc:
                self.sensor_last_error = str(exc)
                quality['reason'] = 'sensor_recovery_failed:' + str(exc)[:300]
                self.get_logger().error(quality['reason'])
            else:
                self.sensor_recovery_pending = False
                self.gyro_calibrated = not self.gyro_auto
                self.calib_last_reason = 'waiting_for_stationary_window_after_recovery'
                quality['reason'] = 'sensor_reinitialized_waiting_for_new_samples'
                self.get_logger().warning('IMU reinitialized; old gyro bias and heading filter discarded')
        quality.update(sensor_recovery_pending=self.sensor_recovery_pending,
                       sensor_recovery_attempts=self.sensor_recovery_attempts,
                       sensor_last_error=self.sensor_last_error)
        self.publish_quality(quality)

    def update_gyro_calibration(self, acc, gyro):
        if self.gyro_calibrated:
            return
        now = time.monotonic()
        self.calib_samples.append((now, tuple(acc), tuple(gyro)))
        while self.calib_samples and now - self.calib_samples[0][0] > self.gyro_calib_seconds:
            self.calib_samples.popleft()
        values = list(self.calib_samples)
        if len(values) < 5 or now - values[0][0] < self.gyro_calib_seconds - 0.25:
            self.calib_last_reason = 'collecting_stationary_samples'
            return
        gmeans = [mean(item[2][j] for item in values) for j in range(3)]
        gstd = [pstdev(item[2][j] for item in values) for j in range(3)]
        anorms = [norm(item[1]) for item in values]
        acc_std = pstdev(anorms)
        if (not 0.80 <= mean(anorms) <= 1.20 or
                acc_std > self.acc_std_limit or
                max(gstd) > self.gyro_std_limit or
                max(abs(x) for x in gmeans) > self.gyro_mean_limit):
            self.calib_last_reason = 'stationarity_check_failed'
            return
        self.gyro_bias = tuple(gmeans)
        self.gyro_calibrated = True
        self.calib_last_reason = 'calibrated_stationary'
        self.get_logger().info('Gyro bias calibrated [deg/s]: ' + str([round(x, 4) for x in self.gyro_bias]))

    def tick(self):
        quality = {
            'hardware_filter_registers': dict(self.sensor.filter_registers),
            'gravity_tau_sec': self.gravity_filter.tau,
            'gravity_accel_smoothing_sec': self.gravity_filter.accel_tau,
            'imu_ok': False, 'mag_ok': False, 'heading_valid': False,
            'heading_preview_magnetic_deg': None, 'heading_preview_true_deg': None,
            'reason': 'not_measured',
            'gyro_calibrated': self.gyro_calibrated,
            'gyro_bias_dps': [round(x, 4) for x in self.gyro_bias],
            'gyro_calibration_reason': self.calib_last_reason,
            'axes_verified': self.axes_verified,
            'declination_configured': self.declination_configured,
            'declination_deg': self.declination,
            'heading_offset_deg': self.horizontal_calibration['correction'] if self.horizontal_calibration else self.heading_offset,
            'heading_correction_source': 'empirical_geographic_north' if self.horizontal_calibration else 'declination_and_mounting',
            'declination_applied': self.horizontal_calibration is None,
            'mag_validation_scope': 'horizontal_only' if self.horizontal_calibration else '3d',
            'mag_z_calibrated': self.calibration is not None,
            'use_2d_horizontal_compass': self.use_2d_compass,
            'calibration_loaded': self.calibration is not None or self.horizontal_calibration is not None,
            'mag_norm_ut': None, 'mag_horizontal_ut': None,
            'sensor_recovery_pending': self.sensor_recovery_pending,
            'sensor_recovery_attempts': self.sensor_recovery_attempts,
            'sensor_last_error': self.sensor_last_error,
        }
        if self.sensor_recovery_pending:
            self.recover_sensor(quality)
            return
        stamp = self.get_clock().now().to_msg()
        acc = (0.0, 0.0, 1.0)
        try:
            ax, ay, az, gx, gy, gz = self.sensor.read_accelerometer_gyro_data()
            acc = apply_axis_map((ax, ay, az), self.acc_axes)
            gyro = apply_axis_map((gx, gy, gz), self.acc_axes)
            if not all(math.isfinite(x) for x in acc + gyro):
                raise ValueError('Nonfinite accelerometer/gyroscope')
            self.update_gyro_calibration(acc, gyro)
            quality['gyro_calibrated'] = self.gyro_calibrated
            quality['gyro_bias_dps'] = [round(x, 4) for x in self.gyro_bias]
            quality['gyro_calibration_reason'] = self.calib_last_reason

            corrected_gyro = tuple(gyro[i] - self.gyro_bias[i] for i in range(3))
            gravity = self.gravity_filter.update(acc, tuple(math.radians(v) for v in corrected_gyro), time.monotonic())
            m = Imu()
            m.header.stamp = stamp
            m.header.frame_id = 'imu_link'
            m.orientation_covariance[0] = -1.0
            m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = (math.radians(v) for v in corrected_gyro)
            m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z = (v * 9.80665 for v in acc)
            self.imu_pub.publish(m)
            quality['imu_ok'] = True
            quality['acc_norm_g'] = round(norm(acc), 3)

            if self.heading_accel_lp is None:
                self.heading_accel_lp = tuple(acc)
            else:
                a = self.heading_accel_alpha
                self.heading_accel_lp = tuple((1 - a) * self.heading_accel_lp[i] + a * acc[i] for i in range(3))
        except (RuntimeError, OSError, ValueError) as exc:
            self.request_sensor_recovery(quality, 'imu_error:' + str(exc)[:300])
            self.publish_quality(quality)
            return

        try:
            raw = self.sensor.read_magnetometer_data()
            if not all(math.isfinite(v) for v in raw):
                raise ValueError('Nonfinite magnetic field')
            if self.horizontal_calibration:
                corrected = calibrated_horizontal_field(raw, self.horizontal_calibration)
                ref = self.horizontal_calibration['reference']
                spread = self.horizontal_calibration['spread']
            elif self.calibration:
                off, scale, ref, spread = self.calibration
                corrected = calibrated_field(raw, off, scale)
            else:
                corrected = tuple(float(v) for v in raw)
            field = apply_axis_map(corrected, self.mag_axes)
            mag_norm = norm(field)
            mag_horiz = math.hypot(field[0], field[1])
            quality['mag_ok'] = True
            quality['mag_norm_ut'] = round(mag_norm, 2)
            quality['mag_horizontal_ut'] = round(mag_horiz, 2)
            quality['mag_raw_ut'] = [round(float(v), 2) for v in raw]
            quality['mag_body_ut'] = [round(float(v), 2) for v in field]

            magmsg = MagneticField()
            magmsg.header.stamp = stamp
            magmsg.header.frame_id = 'imu_link'
            magmsg.magnetic_field.x, magmsg.magnetic_field.y, magmsg.magnetic_field.z = (v * 1e-6 for v in field)
            self.mag_pub.publish(magmsg)

            calibration_present = self.calibration is not None or self.horizontal_calibration is not None
            calibration_ok = bool(calibration_present and spread <= self.max_variation)
            validated_field = mag_horiz if self.horizontal_calibration else mag_norm
            field_min = ref * (1.0 - self.max_deviation) if calibration_present else None
            field_max = ref * (1.0 + self.max_upper_deviation) if calibration_present else None
            field_ok = bool(calibration_present and field_min <= validated_field <= field_max)
            quality['mag_validation_min_ut'] = field_min
            quality['mag_validation_max_ut'] = field_max
            acc_norm = norm(acc)
            raw_tilt_deg = math.degrees(math.acos(max(-1.0, min(1.0, acc[2] / acc_norm)))) if acc_norm > 0 else 180.0
            gravity_ok = bool(gravity['valid'] and self.gyro_calibrated)
            tilt_deg = gravity['tilt_deg'] if self.use_2d_compass and gravity_ok else raw_tilt_deg
            quality['tilt_raw_deg'] = round(raw_tilt_deg, 2)
            quality['tilt_deg'] = round(tilt_deg, 2)
            quality['gravity_estimate_valid'] = gravity_ok
            quality['gravity_accel_used'] = gravity['accel_used']
            quality['gravity_accel_age_sec'] = gravity['accel_age_sec']
            quality['mag_field_valid'] = field_ok
            quality['calibration_valid'] = calibration_ok
            if not calibration_ok:
                quality['reason'] = 'calibration_missing_or_invalid'
            elif not field_ok:
                quality['reason'] = 'magnetic_field_out_of_range'
            elif self.use_2d_compass and not gravity_ok:
                quality['reason'] = 'gravity_estimate_invalid'
            elif self.use_2d_compass and tilt_deg > 15.0:
                quality['reason'] = '2d_compass_tilt_exceeded'
            elif (gravity_ok if self.use_2d_compass else 0.75 <= acc_norm <= 1.25) and mag_horiz >= 5.0:
                if self.use_2d_compass:
                    raw_mag_deg = compass_2d_deg(field)
                else:
                    raw_mag_deg = tilt_compass_deg(self.heading_accel_lp, field)

                rad = math.radians(raw_mag_deg)
                a = self.heading_accel_alpha
                if self.heading_sin_lp is None:
                    self.heading_sin_lp, self.heading_cos_lp = math.sin(rad), math.cos(rad)
                else:
                    self.heading_sin_lp = (1 - a) * self.heading_sin_lp + a * math.sin(rad)
                    self.heading_cos_lp = (1 - a) * self.heading_cos_lp + a * math.cos(rad)
                preliminary = math.degrees(math.atan2(self.heading_sin_lp, self.heading_cos_lp)) % 360.0

                quality['heading_preview_magnetic_deg'] = round(preliminary, 2)
                correction = (self.horizontal_calibration['correction'] if self.horizontal_calibration
                              else self.declination + self.heading_offset)
                true_deg = (preliminary + correction) % 360.0
                quality['heading_preview_true_deg'] = round(true_deg, 2)

                if self.axes_verified and (self.horizontal_calibration is not None or self.declination_configured):
                    quality['heading_valid'] = True
                    quality['reason'] = 'heading_validated_configuration'
                    self.heading_pub.publish(Float64(data=quality['heading_preview_true_deg']))
                else:
                    quality['reason'] = 'axes_or_declination_flag_false'
            else:
                quality['reason'] = 'tilt_or_horizontal_field_out_of_range'
        except (RuntimeError, OSError, ValueError, ZeroDivisionError) as exc:
            self.request_sensor_recovery(quality, 'mag_error:' + str(exc)[:300])

        self.publish_quality(quality)

    def publish_quality(self, quality):
        quality['stamp_unix'] = time.time()
        self.valid_pub.publish(Bool(data=quality['heading_valid']))
        self.quality_pub.publish(String(data=json.dumps(quality, allow_nan=False)))

    def destroy_node(self):
        self.sensor.close()
        return super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ImuMagNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

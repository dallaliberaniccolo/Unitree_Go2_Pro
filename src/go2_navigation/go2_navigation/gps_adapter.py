"""GNSS NMEA adapter based on original gps_neo_m8m.py from the thesis.
Requires pyserial and pynmea2, and serial device accessible from container.
The RMC and GGA must BOTH be recent and valid. No inferred accuracy.
"""
import json
import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import String

class GPSAdapter(Node):
    def __init__(self):
        super().__init__('go2_gps_adapter')
        self.declare_parameter('serial_port', '/dev/serial0')
        self.declare_parameter('baudrate', 9600)
        self.declare_parameter('max_age_sec', 2.5)
        self.max_age = float(self.get_parameter('max_age_sec').value)
        if self.max_age <= 0:
            raise ValueError('max_age_sec must be positive')
        try:
            import serial
            import pynmea2
        except ImportError as exc:
            raise RuntimeError('GPSAdapter requires pyserial and pynmea2 in Docker') from exc
        self.nmea = pynmea2
        self.serial = serial.Serial(str(self.get_parameter('serial_port').value),
                                    int(self.get_parameter('baudrate').value), timeout=0.25)
        self.lock = threading.Lock()
        self.rmc = None
        self.gga = None
        self.stop_event = threading.Event()
        self.reader = threading.Thread(target=self.read_loop, daemon=True)
        self.reader.start()
        self.pub_fix = self.create_publisher(NavSatFix, '/gps/fix', qos_profile_sensor_data)
        self.pub_quality = self.create_publisher(String, '/gps/quality', qos_profile_sensor_data)
        self.create_timer(0.25, self.tick)
        self.get_logger().info('GNSS NMEA adapter active. No commands sent to Go2')

    def read_loop(self):
        while not self.stop_event.is_set():
            try:
                raw = self.serial.readline().decode('ascii', 'ignore').strip()
                if not raw.startswith('$'):
                    continue
                msg = self.nmea.parse(raw, check=True)
                now = time.monotonic()
                typ = getattr(msg, 'sentence_type', '')
                if typ == 'GGA':
                    quality = int(msg.gps_qual or 0)
                    count = int(msg.num_sats or 0)
                    with self.lock:
                        self.gga = (quality, count, now)
                elif typ == 'RMC':
                    status = str(msg.status or '')
                    lat, lon = float(msg.latitude), float(msg.longitude)
                    if not (math.isfinite(lat) and math.isfinite(lon) and
                            -90 <= lat <= 90 and -180 <= lon <= 180):
                        continue
                    with self.lock:
                        self.rmc = (status == 'A', lat, lon, now)
            except (ValueError, TypeError, AttributeError, self.nmea.ParseError):
                continue
            except OSError as exc:
                self.get_logger().error('GPS serial read failed: ' + str(exc))
                break

    def tick(self):
        now = time.monotonic()
        with self.lock:
            rmc = self.rmc
            gga = self.gga
        quality, sats, gga_age = (gga[0], gga[1], now - gga[2]) if gga else (0, 0, None)
        rmc_age = now - rmc[3] if rmc else None
        valid = bool(rmc and rmc[0] and rmc_age <= self.max_age
                     and gga and gga[0] > 0 and gga_age <= self.max_age)
        q = {'fix_valid': valid, 'gga_quality': quality, 'satellites_used': sats,
             'gga_age_s': round(gga_age, 3) if gga_age is not None else None,
             'rmc_age_s': round(rmc_age, 3) if rmc_age is not None else None}
        self.pub_quality.publish(String(data=json.dumps(q)))
        if valid:
            msg = NavSatFix()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'gps_link'
            msg.status.status = NavSatStatus.STATUS_FIX
            msg.status.service = NavSatStatus.SERVICE_GPS
            msg.latitude, msg.longitude = rmc[1], rmc[2]
            msg.altitude = float('nan')  # No verified GGA altitude aligned with RMC
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
            self.pub_fix.publish(msg)

    def destroy_node(self):
        self.stop_event.set()
        self.reader.join(timeout=1.0)
        self.serial.close()
        return super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = GPSAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

"""NEO-M8M via UART in ROS 2: /gps/fix and /gps/quality. Never moves robot."""
import json
import os
import termios
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix,NavSatStatus
from std_msgs.msg import String
from .gps_protocol import parse_nmea

BAUDS={9600:termios.B9600,19200:termios.B19200,38400:termios.B38400,
       57600:termios.B57600,115200:termios.B115200}

def open_serial(path,baud):
    fd=os.open(path,os.O_RDONLY|os.O_NOCTTY|os.O_NONBLOCK)
    try:
        attr=termios.tcgetattr(fd)
        attr[0]=0; attr[1]=0; attr[2]=termios.CS8|termios.CLOCAL|termios.CREAD
        attr[3]=0; attr[4]=BAUDS[baud]; attr[5]=BAUDS[baud]
        attr[6][termios.VMIN]=0; attr[6][termios.VTIME]=0
        termios.tcsetattr(fd,termios.TCSANOW,attr)
    except Exception:
        os.close(fd); raise
    return fd

class GPSNode(Node):
    def __init__(self):
        super().__init__('go2_gps_sensor')
        self.declare_parameter('serial_port','/dev/gps')
        self.declare_parameter('baudrate',9600)
        self.declare_parameter('max_age_sec',2.5)
        port=str(self.get_parameter('serial_port').value)
        baud=int(self.get_parameter('baudrate').value)
        self.max_age=float(self.get_parameter('max_age_sec').value)
        if baud not in BAUDS or self.max_age<=0:
            raise ValueError('Unsupported baudrate or max_age_sec')
        self.fd=open_serial(port,baud)
        self.buffer=b''
        self.rmc=None
        self.gga=None
        self.good_lines=0
        self.invalid_lines=0
        self.fix_pub=self.create_publisher(NavSatFix,'/gps/fix',qos_profile_sensor_data)
        self.quality_pub=self.create_publisher(String,'/gps/quality',qos_profile_sensor_data)
        self.create_timer(.05,self.poll)
        self.create_timer(.25,self.publish_status)
        self.get_logger().info(f'GNSS UART {port} @ {baud}. SOLA LETTURA.')

    def poll(self):
        for _ in range(12):
            try:
                block=os.read(self.fd,1024)
            except BlockingIOError:
                return
            except OSError as e:
                self.get_logger().error(f'GPS UART error: {e}')
                self.rmc=None; self.gga=None
                return
            if not block:
                return
            self.buffer+=block
            if len(self.buffer)>8192:
                self.buffer=self.buffer[-2048:]
            while b'\n' in self.buffer:
                line,self.buffer=self.buffer.split(b'\n',1)
                parsed=parse_nmea(line)
                if parsed is None:
                    self.invalid_lines+=1; continue
                self.good_lines+=1
                now=time.monotonic()
                if parsed[0]=='GGA':
                    self.gga=(parsed[1],parsed[2],now)
                elif parsed[0]=='RMC':
                    self.rmc=(parsed[1],parsed[2],parsed[3],now)

    def publish_status(self):
        now=time.monotonic()
        gga=self.gga; rmc=self.rmc
        ga=round(now-gga[2],3) if gga else None
        ra=round(now-rmc[3],3) if rmc else None
        valid=bool(gga and rmc and gga[0]>0 and rmc[0]
                   and ga<=self.max_age and ra<=self.max_age)
        status={'fix_valid':valid,'gga_quality':gga[0] if gga else 0,
                'satellites_used':gga[1] if gga else None,
                'gga_age_s':ga,'rmc_age_s':ra,
                'nmea_ok_lines':self.good_lines,'nmea_invalid_lines':self.invalid_lines}
        self.quality_pub.publish(String(data=json.dumps(status)))
        if valid:
            m=NavSatFix()
            m.header.stamp=self.get_clock().now().to_msg(); m.header.frame_id='gps_link'
            m.status.status=NavSatStatus.STATUS_FIX; m.status.service=NavSatStatus.SERVICE_GPS
            m.latitude=float(rmc[1]);m.longitude=float(rmc[2]);m.altitude=float('nan')
            m.position_covariance_type=NavSatFix.COVARIANCE_TYPE_UNKNOWN
            self.fix_pub.publish(m)

    def destroy_node(self):
        os.close(self.fd)
        return super().destroy_node()

def main(args=None):
    rclpy.init(args=args); node=None
    try:
        node=GPSNode();rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node: node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

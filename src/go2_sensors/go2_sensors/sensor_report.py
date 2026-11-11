"""Read-only CLI ROS subscriber for latest sensor/navigation values."""
import json
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu,NavSatFix,MagneticField
from std_msgs.msg import String,Bool

class Report(Node):
    def __init__(self):
        super().__init__('go2_sensor_report')
        self.declare_parameter('duration_sec',12.0)
        self.data={}
        for topic,typ,qos in [
            ('/gps/fix',NavSatFix,qos_profile_sensor_data),
            ('/gps/quality',String,qos_profile_sensor_data),
            ('/imu/data',Imu,qos_profile_sensor_data),
            ('/imu/mag',MagneticField,qos_profile_sensor_data),
            ('/imu/quality',String,qos_profile_sensor_data),
            ('/navigation/status',String,10),
            ('/mission/status',String,10),
            ('/navigation/heading_valid',Bool,10)]:
            self.create_subscription(typ,topic,lambda m,k=topic:self.on_msg(k,m),qos)

    def on_msg(self,k,m):
        if k in ('/gps/quality','/imu/quality','/navigation/status','/mission/status'):
            try:val=json.loads(m.data)
            except (ValueError,TypeError):val={'invalid_json':True}
        elif k=='/gps/fix':val={'lat':m.latitude,'lon':m.longitude}
        elif k=='/imu/data':val={'ax_m_s2':round(m.linear_acceleration.x,2),
                                'az_m_s2':round(m.linear_acceleration.z,2)}
        elif k=='/imu/mag':val={'mag_x_uT':round(m.magnetic_field.x*1e6,1)}
        else:val=bool(m.data)
        self.data[k]=(val,time.monotonic())

def main(args=None):
    rclpy.init(args=args);node=Report()
    try:
        dur=float(node.get_parameter('duration_sec').value)
        start=time.monotonic()
        while time.monotonic()-start < dur:
            rclpy.spin_once(node,timeout_sec=.2)
        print('=== RAPPORTO ROS SENSORI (nessun comando al robot) ===')
        now=time.monotonic()
        for k in ['/gps/quality','/gps/fix','/imu/data','/imu/mag','/imu/quality',
                  '/navigation/heading_valid','/navigation/status','/mission/status']:
            if k not in node.data:
                print(f'{k}: NESSUN MESSAGGIO');continue
            v,t=node.data[k]
            age=round(now-t,2)
            print(f'{k}: age={age}s | {json.dumps(v,ensure_ascii=False)}')
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

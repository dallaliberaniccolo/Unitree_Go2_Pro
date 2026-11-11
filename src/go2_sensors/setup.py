from setuptools import setup
from glob import glob
name='go2_sensors'
setup(name=name, version='0.1.0', packages=[name],
      data_files=[('share/ament_index/resource_index/packages',['resource/'+name]),
                  ('share/'+name,['package.xml']),
                  ('share/'+name+'/launch',glob('launch/*.launch.py')),
                  ('share/'+name+'/config',glob('config/*.yaml'))],
      install_requires=['setuptools'], zip_safe=True,
      maintainer='tesi',maintainer_email='noreply@example.com',
      description='GNSS and IMU/magnetometer ROS field diagnostics', license='Apache-2.0',
      entry_points={'console_scripts':[
          'gps_node=go2_sensors.gps_node:main',
          'imu_mag_node=go2_sensors.imu_mag_node:main',
          'sensor_report=go2_sensors.sensor_report:main',
      ]})

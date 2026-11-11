from glob import glob
from setuptools import setup
package_name = 'go2_navigation'
setup(name=package_name, version='0.2.0', packages=[package_name],
      data_files=[('share/ament_index/resource_index/packages', ['resource/' + package_name]),
                  ('share/' + package_name, ['package.xml']),
                  ('share/' + package_name + '/launch', glob('launch/*.launch.py'))],
      install_requires=['setuptools'], zip_safe=True, maintainer='tesi',
      maintainer_email='noreply@example.com', description='Read-only navigation',
      license='Apache-2.0',
      entry_points={'console_scripts': [
          'navigation_node = go2_navigation.node:main',
          'gps_adapter_node = go2_navigation.gps_adapter:main',
      ]})

from setuptools import setup
package_name='go2_mission_manager'
setup(name=package_name, version='0.4.0', packages=[package_name],
      data_files=[('share/ament_index/resource_index/packages', ['resource/' + package_name]),
                  ('share/' + package_name, ['package.xml'])],
      install_requires=['setuptools'], zip_safe=True, maintainer='tesi',
      maintainer_email='noreply@example.com', description='Autonomous waypoint mission manager with preflight and recovery',
      license='Apache-2.0',
      entry_points={'console_scripts': ['mission_manager_node = go2_mission_manager.node:main']})

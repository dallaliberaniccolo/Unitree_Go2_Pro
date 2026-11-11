from setuptools import find_packages, setup
setup(name='go2_bridge', version='0.1.0', packages=find_packages(),
      data_files=[('share/ament_index/resource_index/packages', ['resource/go2_bridge']),
                  ('share/go2_bridge', ['package.xml'])],
      install_requires=['setuptools'], zip_safe=True,
      maintainer='Go2 project maintainers', maintainer_email='maintainer@example.com',
      description='Go2 ROS 2 WebRTC bridge', license='MIT',
      entry_points={'console_scripts': ['bridge = go2_bridge.node:main']})


from setuptools import find_packages, setup

setup(
    name='go2_bridge',
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/go2_bridge']
        ),
        ('share/go2_bridge', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Matteo Finco and Niccolò Dalla Libera',
    author_email='matfinco@gmail.com, dallaliberaniccolo@gmail.com',
    maintainer='Matteo Finco and Niccolò Dalla Libera',
    maintainer_email='matfinco@gmail.com, dallaliberaniccolo@gmail.com',
    description='ROS 2 WebRTC bridge for the Unitree Go2 Pro robot',
    license='MIT',
    entry_points={
        'console_scripts': [
            'bridge = go2_bridge.node:main',
        ],
    },
)

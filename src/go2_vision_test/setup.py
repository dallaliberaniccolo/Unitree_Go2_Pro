from glob import glob
from setuptools import setup

package_name = 'go2_vision_test'
setup(
    name=package_name, version='0.1.0', packages=[package_name],
    data_files=[('share/ament_index/resource_index/packages', ['resource/' + package_name]),
                ('share/' + package_name, ['package.xml']),
                ('share/' + package_name + '/launch', glob('launch/*.launch.py'))],
    install_requires=['setuptools'], zip_safe=True, maintainer='tesi',
    maintainer_email='noreply@example.com', description='Isolated video wolf detection test',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'video_camera_node = go2_vision_test.video_node:main',
        'video_test_inputs_node = go2_vision_test.inputs_node:main',
    ]},
)

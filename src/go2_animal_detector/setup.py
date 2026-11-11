from setuptools import setup

package_name = 'go2_animal_detector'
setup(
    name=package_name, version='0.1.0', packages=[package_name],
    data_files=[('share/ament_index/resource_index/packages', ['resource/' + package_name]),
                ('share/' + package_name, ['package.xml', 'requirements-inference.txt'])],
    install_requires=['setuptools'], zip_safe=True, maintainer='tesi',
    maintainer_email='noreply@example.com', description='Mission-controlled wolf detector',
    license='Apache-2.0',
    entry_points={'console_scripts': ['animal_detector_node = go2_animal_detector.node:main']},
)

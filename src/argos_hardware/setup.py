from setuptools import find_packages, setup

package_name = 'argos_hardware'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zimchaa',
    maintainer_email='zimchaa@argos-ros.local',
    description='ARGOS hardware driver nodes',
    license='MIT',
    entry_points={
        'console_scripts': [
            'hardware_bridge = argos_hardware.hardware_bridge_node:main',
            'imu_node = argos_hardware.imu_node:main',
            'flotilla_node = argos_hardware.flotilla_node:main',
            'ahrs_node = argos_hardware.ahrs_node:main',
            'sonar_node = argos_hardware.sonar_node:main',
            'ir_node = argos_hardware.ir_node:main',
            'camera_node = argos_hardware.camera_node:main',
            'control_panel = argos_hardware.control_panel_node:main',
        ],
    },
)

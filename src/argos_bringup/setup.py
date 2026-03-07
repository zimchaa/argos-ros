from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'argos_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zimchaa',
    maintainer_email='zimchaa@argos-ros.local',
    description='ARGOS robot launch files',
    license='MIT',
    entry_points={'console_scripts': []},
)

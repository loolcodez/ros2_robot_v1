import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'proximity_sensor'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools',
                      'smbus2',
                      'vl53l1x',
                      'python-periphery'],
    zip_safe=True,
    maintainer='orangepi',
    maintainer_email='huuhaaboxi@protonmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'proximity_sensor_node = proximity_sensor.proximity_sensor_node:main'
        ],
    },
)

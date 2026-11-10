from setuptools import setup
from glob import glob
import os

package_name = 'uav_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py'),
        ),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml'),
        ),
        (
            os.path.join('share', package_name, 'worlds'),
            glob('worlds/*'),
        ),
        (
            os.path.join('share', package_name, 'models'),
            glob('models/*'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='SIVA_SAI',
    maintainer_email='sivasai1232006@gmail.com',
    description='Python-based ROS 2 control for two PX4 UAVs.',
    license='TODO',
    entry_points={
        'console_scripts': [
            'controller = uav_control.controller:main',
        ],
    },
)

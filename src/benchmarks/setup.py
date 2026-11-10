from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'benchmarks'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'LICENSE']),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'docs'),
            glob('docs/*.md')),
    ],
    install_requires=['setuptools', 'PyYAML', 'PuLP'],
    zip_safe=True,
    maintainer='siva',
    maintainer_email='sivasai1232006@gmail.com',
    description='Isolated UAV task allocator benchmarks',
    license='MIT',
    entry_points={
        'console_scripts': [
            'run_benchmark = benchmarks.runner:main',
            'compare_benchmarks = benchmarks.compare:main',
        ],
    },
)

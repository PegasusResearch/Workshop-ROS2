import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'drone_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Marcelo Jacinto',
    maintainer_email='mjacinto@isr.tecnico.ulisboa.pt',
    description='PID drone controller for position tracking',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drone_controller = drone_control.drone_controller_node:main',
        ],
    },
)

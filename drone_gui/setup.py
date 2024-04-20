import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'drone_gui'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'assets'), glob('assets/*.png')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='marcelo',
    maintainer_email='mjacinto@isr.tecnico.ulisboa.pt',
    description='',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drone_gui = drone_gui.drone_gui_node:main',
        ],
    },
)

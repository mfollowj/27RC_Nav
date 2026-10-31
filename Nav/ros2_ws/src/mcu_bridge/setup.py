import os
from glob import glob

from setuptools import setup

package_name = 'mcu_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zx',
    maintainer_email='zx@example.com',
    description='RViz goal + robot pose -> MCU serial position frames (0x7E/0x21).',
    license='MIT',
    entry_points={
        'console_scripts': [
            'mcu_bridge_node = mcu_bridge.position_bridge_node:main',
            'mcu_sniff = mcu_bridge.sniff:main',
        ],
    },
)

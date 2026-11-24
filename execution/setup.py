from setuptools import setup
import os
from glob import glob

package_name = 'execution'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py') + glob('launch/*.launch.xml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml') + glob('config/*.rviz')),
    ],
    install_requires=['setuptools', 'paho-mqtt>=1.6.0'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your-email@example.com',
    description='Fleet adapter for AMR robots (Execution Node fleet; MQTT communication)',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rmf_integrated_fleet_adapter = execution.rmf_integrated_fleet_adapter:main',
            'map_image_publisher = execution.map_image_publisher:main',
            'task_state_bridge = execution.execution_task_state_bridge:main',
            'continuous_inbound_delivery = execution.continuous_inbound_delivery:main',
        ],
    },
)

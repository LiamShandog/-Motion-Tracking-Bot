from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'motion_tracking_bot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # install launch file
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # install config file
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'pigpio>=1.78'],
    zip_safe=True,
    maintainer='liam',
    maintainer_email='liamshan13@gmail.com',
    description='Motion tracking robot using IR sensor, servo, and speaker',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ir_sensor_node = motion_tracking_bot.ir_sensor_node:main',
            'speaker_node = motion_tracking_bot.speaker_node:main',
        ],
    },
)

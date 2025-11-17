from setuptools import find_packages, setup

package_name = 'motion_tracking_bot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='liam',
    maintainer_email='liam@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ir_sensor_node = motion_tracking_bot.ir_sensor_node:main'
            'speaker_node = motion_tracking_bot.speaker_node:main',
        ],
    },
)

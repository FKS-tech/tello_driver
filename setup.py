from setuptools import setup
from glob import glob
import os

package_name = 'tello_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Seu Nome',
    maintainer_email='seu_email@exemplo.com',
    description='Driver ROS 2 em Python para o Tello',
    license='MIT',
    entry_points={
        'console_scripts': [
            'joy_node = tello_driver.joy_node:main',
            'stream_node = tello_driver.stream_node:main',
            'telemetry_node = tello_driver.telemetry_node:main',
            'vision_node = tello_driver.vision_node:main',
            'visual_servo_node = tello_driver.visual_servo_node:main',
            'qr_node = tello_driver.qr_node:main',
            'command_mux_node = tello_driver.command_mux_node:main',
        ],
    },
)

from setuptools import setup
import os
from glob import glob

package_name = 'orbslam3_ros2'

setup(
    name=package_name,
    version='1.0.0',
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
    maintainer='Daniel Xie',
    maintainer_email='danxie2001@gmail.com',
    description='ORB SLAM 3 ROS2 Wrapper',
    license='MIT',
)

from setuptools import setup
import os
from glob import glob

package_name = 'map_processor'

setup(
    name=package_name,
    version='0.0.1',
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
    description='Map Processing Node',
    license='MIT',
    entry_points={
        'console_scripts': [
            'map_processor = map_processor.map_processor:main'
        ],
    },
)

from setuptools import find_packages, setup
from glob import glob
import os
package_name = 'under_water_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name+'/launch', glob('launch/*.launch.py')),
    ('share/' + package_name+'/config', glob('config/*.yaml')),
],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dsy',
    maintainer_email='820145912@qq.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            
            'thrust_control = under_water_robot.thrust_control:main',
        ],
    },
)

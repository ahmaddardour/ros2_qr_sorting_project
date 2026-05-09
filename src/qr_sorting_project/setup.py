from glob import glob
import os
from setuptools import setup

package_name = 'qr_sorting_project'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ahmad',
    maintainer_email='ahmad.nedal92@yahoo.com',
    description='QR-based ROS 2 + MoveIt 2 sorting pipeline.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'main_node = qr_sorting_project.main_node:main',
            'moveit_controller = qr_sorting_project.moveit_controller:main',
            'scene_publisher = qr_sorting_project.scene_publisher:main',
        ],
    },
)

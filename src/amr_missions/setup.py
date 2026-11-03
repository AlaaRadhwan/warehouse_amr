from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'amr_missions'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alaa-radhwan',
    maintainer_email='alaa.radhwanmustafa@gmail.com',
    description='Station-based mission layer: robot_executor (NavigateToPose and Nav2 DockRobot calls) and mission_manager (multi-station missions) over amr_interfaces actions.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'robot_executor = amr_missions.robot_executor:main',
            'mission_manager = amr_missions.mission_manager:main',
        ],
    },
)

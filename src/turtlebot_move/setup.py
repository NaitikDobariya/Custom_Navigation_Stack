from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'turtlebot_move'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='naitik',
    maintainer_email='123naitik456@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'custom_localizer = turtlebot_move.custom_localizer:main',
            'a_star_planner = turtlebot_move.a_star_planner:main',
            'path_follower = turtlebot_move.path_follower:main',
        ],
    },
)


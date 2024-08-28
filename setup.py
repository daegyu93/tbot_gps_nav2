from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'tbot_gps_nav2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dghwang',
    maintainer_email='daegyu@telelian.com',
    description='tbot gps nav2 ',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'interactive_waypoint_follower = nav2_gps_waypoint_follower_demo.interactive_waypoint_follower:main',
        ],
    },
)

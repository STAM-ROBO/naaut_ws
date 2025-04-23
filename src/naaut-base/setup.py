from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'naaut-base'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]), ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'params'), glob("params/*.yaml")),
        (os.path.join('share', package_name, 'urdf'), glob("urdf/*.urdf")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='stam',
    maintainer_email='stam@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'logged_waypoint_follower = scripts.logged_waypoint_follower:main',
            'interactive_waypoint_follower = scripts.interactive_waypoint_follower:main',
            'gps_waypoint_logger = scripts.gps_waypoint_logger:main'
        ],
    },
)

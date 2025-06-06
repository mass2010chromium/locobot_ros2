import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'locobot_navigation'

def path(*args):
    return os.path.join(*args)

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
                ['resource/' + package_name]),
        (path('share', package_name), ['package.xml']),
        (path('share', package_name, 'config'), glob(os.path.join('config', '*'))),
        (path('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pyx][yaml]*')))
      ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jing-Chen Peng',
    maintainer_email='jpeng303@gatech.edu',
    description='locobot nav scripts (dr. vela\'s lab version)',
    license='Apache License 2.0',
    tests_require=[],
    entry_points={
        'console_scripts': [
                'depth_image_to_laserscan = locobot_navigation.depth_image_to_laserscan:main',
                'republish_scan = locobot_navigation.republish_scan:main'
        ],
      },
)

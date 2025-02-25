from setuptools import find_packages, setup

import os
from glob import glob

package_name = 'orion_web_interface'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='miguel Angel Gonzalez Rodriguez',
    maintainer_email='miguel_gonzalezr@ieee.org',
    description='Package that launches backend for the ORIOn robot web interface',
    license='BSD-3',
    entry_points={
        'console_scripts': [
            "backend = orion_web_interface.backend:main",
        ],
    },
)

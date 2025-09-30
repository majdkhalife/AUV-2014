from setuptools import find_packages, setup

import os
from glob import glob

package_name = 'propulsion'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('lib/' + package_name, [package_name+'/thrust_mapper_utils.py']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='McGill Robotics',
    maintainer_email='dev@mcgillrobotics.com',
    description='Package for moving the AUV. Designed to abstract away the mechanical implementation details of the propulsion mechanism.',
    license='GPLv3',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'thrust_mapper = propulsion.thrust_mapper:main',
        ],
    },
)

import os
from glob import glob
from setuptools import find_packages, setup
import subprocess

package_name = 'kimchi_map'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']), 
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='arilow',
    maintainer_email='arilow@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'kimchi_map_handler = kimchi_map.kimchi_map_handler:main',
        ],
    },
)

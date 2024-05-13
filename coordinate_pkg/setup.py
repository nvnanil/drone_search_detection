import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'coordinate_pkg'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nvnanil',
    maintainer_email='nvnanil@umd.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        "console_scripts": [
            "coordinate_node = coordinate_pkg.object_camera_coord:main",
            "inertial_node = coordinate_pkg.inertial_camera_coord:main"
        ],
    },
)

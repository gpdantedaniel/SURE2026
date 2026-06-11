from setuptools import find_packages, setup
import os
import glob

package_name = 'bioprinting'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'box.gcode']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jmk',
    maintainer_email='gp.dantedaniel@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "invitro_planner = bioprinting.invitro_planner:main"
        ],
    },
)

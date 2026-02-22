from setuptools import find_packages, setup
from glob import glob

package_name = 'grip_command_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jakob D. Gros',
    maintainer_email='jakob.gros@hs-aalen.de',
    description='Serial gripper interface for Robotiq E-Pick on UR robots',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gripper_node = grip_command_package.gripper_node:main',
        ],
    },
)

from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'test_suite'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Elvis Borges',
    maintainer_email='sivlesegrob@gmail.com',
    description='Test utilities and fake publishers for RecycleBot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fake_rgbd_publisher = test_suite.fake_rgbd_publisher:main',
            'fake_joint_state_publisher = test_suite.fake_joint_state_publisher:main',
        ],
    },
)

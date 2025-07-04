import os
from setuptools import find_packages, setup
from pathlib import Path
from glob import glob



package_name = 'recycle_bot'

# read dependencies from requirements.txt
requirements_path = Path(__file__).parent / 'requirements.txt'
install_requires = ['setuptools']

if requirements_path.exists():
    with open(requirements_path, "r") as f:
        requirements = [
            line.strip() for line in f
            if line.strip() and not line.startswith("#")  # ignore blank lines & comments
        ]
        install_requires.extend(requirements)
        
setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('**/*.yaml', recursive=True)), # install all yaml files in share
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=install_requires,  # add requirements from requirements.txt
    zip_safe=True,
    maintainer='Elvis Borges',
    maintainer_email='todo@todo.todo',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rec_bot_core = recycle_bot.rec_bot_core:main',
            'rec_bot_control = recycle_bot.rec_bot_control:main',
            'rec_bot_vision = recycle_bot.rec_bot_vision:main',
            'rec_bot_smoke = recycle_bot.rec_bot_smoke:main'
        ],
    },
)

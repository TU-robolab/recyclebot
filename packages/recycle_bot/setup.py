import os
from setuptools import find_packages, setup
from pathlib import Path
from glob import glob



package_name = 'recycle_bot'


def config_data_files(pkg):
    """Install config/ preserving its directory structure.

    Per-robot config lives in config/<ur_type>/ (see recycle_bot/robot_profile.py),
    so the same filename exists more than once and the install destination has to
    mirror the source layout rather than collapse into one directory.
    """
    entries = []
    for dirpath, _dirnames, filenames in os.walk('config'):
        payload = [
            os.path.join(dirpath, f)
            for f in filenames
            if f.endswith(('.yaml', '.rviz'))
        ]
        if payload:
            # dirpath is already relative and starts with 'config'
            entries.append((os.path.join('share', pkg, dirpath), payload))
    return entries


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
        # Config is installed per directory, NOT with a single recursive glob.
        # data_files flattens: one ('.../config', glob('config/**/*.yaml')) entry
        # would install config/ur16e/calibration.yaml and
        # config/ur3e/calibration.yaml to the same destination path, and the
        # second would silently overwrite the first.
        *config_data_files(package_name),
        (os.path.join('share', package_name, 'pkg_resources'), glob(os.path.join('pkg_resources', '*'))),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=install_requires,  # add requirements from requirements.txt
    zip_safe=True,
    maintainer='Elvis Borges',
    maintainer_email='elvis@triku.studio',
    description='CV-based pick-and-place pipeline: vision, core, and control nodes for RecycleBot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rec_bot_core = recycle_bot.rec_bot_core:main',
            'rec_bot_control = recycle_bot.rec_bot_control:main',
            'rec_bot_vision = recycle_bot.rec_bot_vision:main',
            'rec_bot_smoke = recycle_bot.rec_bot_smoke:main',
            'launch_gate = recycle_bot.launch_gate:main',
            'rec_bot_viz = recycle_bot.rec_bot_viz:main',
            'check_calibration = recycle_bot.calibration_check:main',
            'check_robot = recycle_bot.robot_identity:main',
        ],
    },
)

from setuptools import find_packages, setup
from pathlib import Path


package_name = 'recycle_bot'

# read dependencies from requirements.txt
requirements_path = Path(__file__).parent / 'requirements.txt'
install_requires = requirements_path.read_text().splitlines() if requirements_path.exists() else []

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'] + install_requires,  # add requirements from requirements.txt
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
            'rec_bot_vision = recycle_bot.rec_bot_vision:main'
        ],
    },
)

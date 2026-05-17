from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'jaka_visual_inspection'


def get_files(directory, pattern='*'):
    """Get only regular files matching pattern in directory."""
    files = glob(os.path.join(directory, pattern))
    return [f for f in files if os.path.isfile(f)]


setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'),
            get_files('launch', '*.py')),
        # Config files
        (os.path.join('share', package_name, 'config'),
            get_files('config')),
        # GUI files
        (os.path.join('share', package_name, 'gui'),
            get_files('gui', '*.py')),
        # GUI data - Profile Previews
        (os.path.join('share', package_name, 'gui', 'VI_appdata', 'Profile_Previews'),
            get_files('gui/VI_appdata/Profile_Previews')),
        # GUI data - Robot positions
        (os.path.join('share', package_name, 'gui', 'VI_appdata', 'Robo_object_positions'),
            get_files('gui/VI_appdata/Robo_object_positions')),
        # World files
        (os.path.join('share', package_name, 'worlds'),
            get_files('worlds', '*.sdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Auto Path Planner for JAKA ZU5 - Visual inspection path planning',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'auto_planner_node = jaka_visual_inspection.auto_planner_node:main',
        ],
    },
)

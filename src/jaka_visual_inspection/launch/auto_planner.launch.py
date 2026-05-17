"""
Launch file for the JAKA ZU5 Auto Path Planner.

This launches only the auto planner node.
MoveIt and the camera driver should be launched separately.

Usage:
    1. First launch MoveIt:
       ros2 launch jaka_zu5_moveit_config demo.launch.py

    2. Then launch the camera:
       ros2 launch realsense2_camera rs_launch.py

    3. Then launch this planner:
       ros2 launch jaka_visual_inspection auto_planner.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('jaka_visual_inspection')
    config_file = os.path.join(pkg_share, 'config', 'config.ini')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=config_file,
            description='Path to the configuration file'
        ),

        Node(
            package='jaka_visual_inspection',
            executable='auto_planner_node',
            name='auto_planner_node',
            output='screen',
            parameters=[{
                'config_file': LaunchConfiguration('config_file'),
            }],
        ),
    ])

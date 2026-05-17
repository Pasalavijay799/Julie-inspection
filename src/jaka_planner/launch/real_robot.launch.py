"""
Launch file for operating the real JAKA robot with MoveIt + RViz.

Combines:
  - jaka_driver (robot connection)
  - robot_state_publisher
  - move_group (MoveIt planning)
  - RViz (visualization and interactive planning)
  - OnRobot 2FG7 gripper driver (optional)

Usage:
  ros2 launch jaka_planner real_robot.launch.py ip:=192.168.0.50 model:=zu5 gripper_ip:=192.168.0.100
"""

import os

import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    model = LaunchConfiguration('model')
    gripper_config_file = os.path.join(
        get_package_share_directory('jaka_driver'),
        'config',
        'onrobot_2fg7_modbus.yaml'
    )

    return LaunchDescription([
        # ── Arguments ──
        DeclareLaunchArgument('ip', default_value='192.168.0.50',
                              description='JAKA robot IP address'),
        DeclareLaunchArgument('model', default_value='zu5',
                              description='JAKA robot model (zu5, zu7, etc.)'),
        DeclareLaunchArgument('use_gripper', default_value='true',
                              description='Enable OnRobot 2FG7 gripper'),
        DeclareLaunchArgument('gripper_ip', default_value='192.168.0.100',
                              description='Gripper Compute Box IP address'),

        LogInfo(msg=["Launching real robot: model=", model,
                     ", ip=", LaunchConfiguration('ip'),
                     ", gripper_ip=", LaunchConfiguration('gripper_ip')]),

        # ── 1. JAKA Robot Driver ──
        Node(
            package='jaka_driver',
            executable='jaka_driver',
            name='jaka_driver',
            output='screen',
            parameters=[{'ip': LaunchConfiguration('ip')}],
        ),

        # ── 2. Robot State Publisher ──
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('jaka_zu5_moveit_config'),
                    'launch', 'rsp.launch.py'
                )
            ),
        ),

        # ── 3. MoveIt move_group ──
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('jaka_zu5_moveit_config'),
                    'launch', 'move_group.launch.py'
                )
            ),
        ),

        # ── 4. RViz ──
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('jaka_zu5_moveit_config'),
                    'launch', 'moveit_rviz.launch.py'
                )
            ),
        ),

        # ── 5. OnRobot 2FG7 Gripper Driver ──
        Node(
            package='jaka_driver',
            executable='onrobot_2fg7_driver.py',
            name='onrobot_2fg7_driver',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_gripper')),
            parameters=[
                gripper_config_file,
                {'gripper_ip': LaunchConfiguration('gripper_ip')},
            ],
        ),
    ])

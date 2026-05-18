"""
real_robot_zed.launch.py
========================
Launches ONLY the ZED 2i camera wrapper for use with the real JAKA ZU5.

Use this alongside the real robot commands:
  Terminal 1:  ros2 launch jaka_planner moveit_server.launch.py ip:=192.168.0.50 model:=zu5 use_gripper:=true gripper_ip:=192.168.0.75 use_camera:=false
  Terminal 2:  ros2 launch jaka_zu5_moveit_config demo.launch.py use_rviz_sim:=false
  Terminal 3:  ros2 launch jaka_visual_inspection real_robot_zed.launch.py
  Terminal 4:  ros2 run jaka_visual_inspection safety_camera_viewer
  Terminal 5:  ros2 run jaka_planner iit_logo_safety_demo.py
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    zed_safety_config = os.path.join(
        get_package_share_directory('jaka_visual_inspection'),
        'config', 'zed2i_safety.yaml'
    )

    zed_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('zed_wrapper'),
                'launch', 'zed_camera.launch.py'
            )
        ),
        launch_arguments={
            'camera_model':              'zed2i',
            'camera_name':               'zed',
            'publish_urdf':              'true',
            'publish_tf':                'true',
            'publish_map_tf':            'true',
            'use_sim_time':              'false',
            'ros_params_override_path':  zed_safety_config,
        }.items(),
    )

    return LaunchDescription([
        zed_camera,
    ])

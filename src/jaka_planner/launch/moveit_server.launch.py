import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    gripper_config_file = os.path.join(
        get_package_share_directory('jaka_driver'),
        'config',
        'onrobot_2fg7_modbus.yaml'
    )

    return LaunchDescription([
        # Declare 'ip' and 'model' arguments
        DeclareLaunchArgument('ip', default_value='192.168.1.50', description='IP address'),
        DeclareLaunchArgument('model', default_value='default_model', description='Model name'),
        DeclareLaunchArgument('use_gripper', default_value='true', description='Enable direct Ethernet gripper node'),
        DeclareLaunchArgument('gripper_ip', default_value='192.168.1.100', description='Gripper IP address'),
        DeclareLaunchArgument('gripper_port', default_value='502', description='Gripper Modbus TCP port'),
        DeclareLaunchArgument('gripper_unit_id', default_value='65', description='Gripper Modbus unit id'),
        DeclareLaunchArgument('use_camera', default_value='true', description='Enable RealSense D435 camera'),

        # Launch the 'moveit_server' node from the 'jaka_planner' package
        Node(
            package='jaka_planner',
            executable='moveit_server',  # the executable to run
            name='moveit_server',
            output='screen',
            parameters=[
                {'ip': LaunchConfiguration('ip')},  # Pass 'ip' parameter
                {'model': LaunchConfiguration('model')}  # Pass 'model' parameter
            ],
        ),

        Node(
            package='jaka_driver',
            executable='onrobot_2fg7_driver.py',
            name='onrobot_2fg7_driver',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_gripper')),
            parameters=[
                gripper_config_file,
                {'gripper_ip': LaunchConfiguration('gripper_ip')},
                {'gripper_port': LaunchConfiguration('gripper_port')},
                {'unit_id': LaunchConfiguration('gripper_unit_id')},
            ],
        ),
        # Intel RealSense D435 camera node
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='camera',
            namespace='camera',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_camera')),
            parameters=[{
                'enable_color': True,
                'enable_depth': True,
                'enable_infra1': False,
                'enable_infra2': False,
                'depth_module.profile': '640x480x30',
                'rgb_camera.profile': '640x480x30',
                'align_depth.enable': True,
                'pointcloud.enable': True,
                'camera_name': 'camera',
            }],
        ),
    ])

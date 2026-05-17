import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Declare the 'ip' argument
        DeclareLaunchArgument('ip', default_value='192.168.1.50', description='IP address'),
        DeclareLaunchArgument('use_gripper', default_value='true', description='Enable direct Ethernet gripper node'),
        DeclareLaunchArgument('gripper_ip', default_value='192.168.1.100', description='Gripper IP address'),
        DeclareLaunchArgument('gripper_port', default_value='502', description='Gripper Modbus TCP port'),
        DeclareLaunchArgument('gripper_unit_id', default_value='65', description='Gripper Modbus unit id'),

        # Print the IP to the log for debugging
        LogInfo(
            msg=["The IP address is: ", LaunchConfiguration('ip')]  # Correct substitution usage
        ),

        # Launch the 'jaka_driver' node with the provided 'ip' parameter
        Node(
            package='jaka_driver',
            executable='jaka_driver',  # the executable to run
            name='jaka_driver',
            output='screen',
            parameters=[{'ip': LaunchConfiguration('ip')}],  # pass the 'ip' parameter
        ),

        Node(
            package='jaka_driver',
            executable='onrobot_2fg7_driver.py',
            name='onrobot_2fg7_driver',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_gripper')),
            parameters=[
                {'gripper_ip': LaunchConfiguration('gripper_ip')},
                {'gripper_port': LaunchConfiguration('gripper_port')},
                {'unit_id': LaunchConfiguration('gripper_unit_id')},
            ],
        ),
    ])

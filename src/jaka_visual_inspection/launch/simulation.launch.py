"""
Simulation launch file for JAKA ZU5 Visual Inspection.

Based on the existing demo_gazebo.launch.py from jaka_zu5_moveit_config,
extended with our custom inspection world and camera bridge.

Usage:
    ros2 launch jaka_visual_inspection simulation.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package paths
    pkg_vi = get_package_share_directory('jaka_visual_inspection')
    pkg_moveit = get_package_share_directory('jaka_zu5_moveit_config')
    pkg_description = get_package_share_directory('jaka_description')
    pkg_gz_sim = get_package_share_directory('ros_gz_sim')

    # World file
    world_file = os.path.join(pkg_vi, 'worlds', 'inspection_world.sdf')

    # URDF via xacro
    xacro_file = os.path.join(pkg_moveit, 'config', 'jaka_zu5.urdf.xacro')
    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ', xacro_file,
        ' use_gazebo:=true',
        ' use_rviz_sim:=false',
        ' initial_positions_file:=',
        os.path.join(pkg_moveit, 'config', 'initial_positions.yaml'),
    ])
    robot_description = ParameterValue(robot_description_content, value_type=str)

    # ===================== Environment for mesh resolution =====================
    gz_resource_path = os.path.dirname(pkg_description)
    set_gz_resource = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=gz_resource_path + ':' + os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')
    )

    # ===================== Ignition Gazebo (using ros_gz_sim like the existing launch) =====================
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'{world_file} -r',
        }.items(),
    )

    # ===================== Robot State Publisher =====================
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
    )

    # ===================== Spawn Robot =====================
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_robot',
        output='screen',
        arguments=[
            '-topic', '/robot_description',
            '-z', '0.0',
        ],
    )

    # ===================== Controllers (chained sequentially) =====================
    # Same pattern as spawn_controllers.launch.py but chained to handle
    # the rendering block from the camera sensor
    spawn_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager-timeout', '120',
            '--switch-timeout', '120',
            '--service-call-timeout', '120',
        ],
        output='screen',
    )

    spawn_arm = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'jaka_zu5_controller',
            '--controller-manager-timeout', '120',
            '--switch-timeout', '60',
            '--service-call-timeout', '60',
        ],
        output='screen',
    )

    spawn_gripper = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'gripper_controller',
            '--controller-manager-timeout', '120',
            '--switch-timeout', '60',
            '--service-call-timeout', '60',
        ],
        output='screen',
    )

    # ===================== Ignition → ROS2 Bridge =====================
    ign_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/camera/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/camera/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            '/camera/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
        ],
        remappings=[
            ('/camera/image', '/camera/color/image_raw'),
            ('/camera/depth_image', '/camera/depth/image_raw'),
            ('/camera/camera_info', '/camera/depth/camera_info'),
        ],
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # ===================== Simulation Camera Optical Frame Shim =====================
    cam_gz_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '-1.5708', '--yaw', '1.5708',
            '--frame-id', 'camera_depth_optical_frame',
            '--child-frame-id', 'camera_link_gz_sensor'
        ],
        output='screen'
    )

    # ===================== MoveIt2 (direct node for log level control) =====================
    from moveit_configs_utils import MoveItConfigsBuilder
    moveit_config = MoveItConfigsBuilder(
        'jaka_zu5', package_name='jaka_zu5_moveit_config'
    ).to_moveit_configs()

    moveit_dict = moveit_config.to_dict()
    moveit_dict.update({
        'publish_robot_description_semantic': True,
        'allow_trajectory_execution': True,
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
        'monitor_dynamics': False,
        'use_sim_time': True,
    })

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[moveit_dict],
        arguments=[
            '--ros-args',
            '--log-level', 'moveit_robot_model.robot_model:=fatal',
            '--log-level', 'tf2_ros:=error',
            '--log-level', 'tf2:=error',
        ],
    )

    # ===================== RViz (manual to ensure use_sim_time) =====================
    rviz_parameters = [
        moveit_config.planning_pipelines,
        moveit_config.robot_description_kinematics,
        {'use_sim_time': True}
    ]
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=[
            '-d', os.path.join(pkg_moveit, 'config', 'moveit.rviz'),
            '--ros-args',
            '--log-level', 'tf2_ros:=error',
            '--log-level', 'tf2:=error',
        ],
        parameters=rviz_parameters,
    )

    # ===================== Chained Event Handlers =====================
    # JSB → arm → gripper → MoveIt + RViz
    jsb_done = RegisterEventHandler(
        OnProcessExit(target_action=spawn_jsb, on_exit=[spawn_arm])
    )
    arm_done = RegisterEventHandler(
        OnProcessExit(target_action=spawn_arm, on_exit=[spawn_gripper])
    )
    gripper_done = RegisterEventHandler(
        OnProcessExit(target_action=spawn_gripper, on_exit=[move_group_node, rviz])
    )

    # ===================== Launch =====================
    return LaunchDescription([
        set_gz_resource,

        # Core
        gazebo,
        robot_state_publisher,
        ign_bridge,
        cam_gz_tf,

        # Spawn robot after Gazebo starts
        TimerAction(period=5.0, actions=[spawn_robot]),

        # Start JSB at t=10 (survives rendering block with 120s timeouts)
        TimerAction(period=10.0, actions=[spawn_jsb]),

        # Chained: JSB → arm → gripper → MoveIt + RViz
        jsb_done,
        arm_done,
        gripper_done,
    ])

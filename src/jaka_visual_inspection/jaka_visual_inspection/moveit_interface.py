"""
MoveIt2 interface module for JAKA Visual Inspection.

Provides motion planning and execution for the JAKA ZU5 robot
using MoveIt2's MoveGroupInterface via action client.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from moveit_msgs.msg import (
    MoveItErrorCodes,
    RobotState,
    Constraints,
    JointConstraint,
    PositionConstraint,
    OrientationConstraint,
)
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.srv import GetPositionIK, GetMotionPlan
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive

import numpy as np
import time


class MoveItInterface:
    """Interface for MoveIt2 planning and execution for JAKA ZU5."""

    PLANNING_GROUP = 'jaka_zu5'
    EEF_LINK = 'Link_6'
    BASE_LINK = 'world'
    JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

    def __init__(self, node):
        """
        Args:
            node: ROS2 node
        """
        self.node = node
        self.logger = node.get_logger()

        # Action client for trajectory execution
        self._execute_client = ActionClient(
            node, ExecuteTrajectory, '/execute_trajectory'
        )

        # Service clients
        self._ik_client = node.create_client(
            GetPositionIK, '/compute_ik'
        )
        self._plan_client = node.create_client(
            GetMotionPlan, '/plan_kinematic_path'
        )

        # Current joint state subscriber
        self._current_joints = None
        self._joint_sub = node.create_subscription(
            JointState, '/joint_states', self._joint_state_callback, 10
        )

        self.logger.info("MoveIt interface initialized for JAKA ZU5")

    def _joint_state_callback(self, msg):
        """Store the latest joint state."""
        self._current_joints = msg

    def get_current_joint_values(self):
        """Get current joint positions."""
        if self._current_joints is None:
            self.logger.warn("No joint state received yet")
            return [0.0] * 6

        # Map joint names to positions
        values = []
        for name in self.JOINT_NAMES:
            if name in self._current_joints.name:
                idx = self._current_joints.name.index(name)
                values.append(self._current_joints.position[idx])
            else:
                values.append(0.0)
        return values

    def go_to_joint_state(self, joint_values, velocity_scaling=0.3,
                          acceleration_scaling=0.3):
        """
        Plan and execute motion to a joint state goal.

        Args:
            joint_values: list of 6 joint angle values (radians)
            velocity_scaling: max velocity scaling factor (0-1)
            acceleration_scaling: max acceleration scaling factor (0-1)

        Returns:
            success: True if motion completed
        """
        self.logger.info(f"Moving to joint state: {[f'{v:.3f}' for v in joint_values]}")

        if not self._plan_client.wait_for_service(timeout_sec=5.0):
            self.logger.error("Planning service not available")
            return False

        # Build motion plan request
        request = GetMotionPlan.Request()
        mp_request = request.motion_plan_request

        mp_request.group_name = self.PLANNING_GROUP
        mp_request.num_planning_attempts = 10
        mp_request.allowed_planning_time = 5.0
        mp_request.max_velocity_scaling_factor = velocity_scaling
        mp_request.max_acceleration_scaling_factor = acceleration_scaling

        # Set joint constraints as goal
        goal_constraints = Constraints()
        for i, name in enumerate(self.JOINT_NAMES):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(joint_values[i])
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            goal_constraints.joint_constraints.append(jc)

        mp_request.goal_constraints.append(goal_constraints)

        # Set start state to current
        if self._current_joints is not None:
            mp_request.start_state.joint_state = self._current_joints

        # Call planning service
        future = self._plan_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)

        if future.result() is None:
            self.logger.error("Planning failed: no result")
            return False

        result = future.result()
        if result.motion_plan_response.error_code.val != MoveItErrorCodes.SUCCESS:
            self.logger.error(
                f"Planning failed with code: {result.motion_plan_response.error_code.val}"
            )
            return False

        # Execute the planned trajectory
        return self._execute_trajectory(result.motion_plan_response.trajectory)

    def go_to_pose_goal(self, pose, velocity_scaling=0.3,
                        acceleration_scaling=0.3):
        """
        Plan and execute motion to a Cartesian pose goal.

        Args:
            pose: [x, y, z, qx, qy, qz, qw] or geometry_msgs.msg.Pose
            velocity_scaling: max velocity scaling factor
            acceleration_scaling: max acceleration scaling factor

        Returns:
            success: True if motion completed
        """
        if isinstance(pose, (list, tuple, np.ndarray)):
            pose_msg = Pose()
            pose_msg.position.x = float(pose[0])
            pose_msg.position.y = float(pose[1])
            pose_msg.position.z = float(pose[2])
            pose_msg.orientation.x = float(pose[3])
            pose_msg.orientation.y = float(pose[4])
            pose_msg.orientation.z = float(pose[5])
            pose_msg.orientation.w = float(pose[6])
        else:
            pose_msg = pose

        self.logger.info(
            f"Moving to pose: [{pose_msg.position.x:.3f}, "
            f"{pose_msg.position.y:.3f}, {pose_msg.position.z:.3f}]"
        )

        if not self._plan_client.wait_for_service(timeout_sec=5.0):
            self.logger.error("Planning service not available")
            return False

        # Build motion plan request
        request = GetMotionPlan.Request()
        mp_request = request.motion_plan_request

        mp_request.group_name = self.PLANNING_GROUP
        mp_request.num_planning_attempts = 10
        mp_request.allowed_planning_time = 5.0
        mp_request.max_velocity_scaling_factor = velocity_scaling
        mp_request.max_acceleration_scaling_factor = acceleration_scaling

        # Set pose constraint as goal
        goal_constraints = Constraints()

        # Position constraint
        pc = PositionConstraint()
        pc.header.frame_id = self.BASE_LINK
        pc.link_name = self.EEF_LINK
        pc.target_point_offset.x = 0.0
        pc.target_point_offset.y = 0.0
        pc.target_point_offset.z = 0.0

        # Define a small bounding region around the target
        bv = SolidPrimitive()
        bv.type = SolidPrimitive.SPHERE
        bv.dimensions = [0.01]  # 1cm tolerance sphere

        bv_pose = Pose()
        bv_pose.position = pose_msg.position
        bv_pose.orientation.w = 1.0

        pc.constraint_region.primitives.append(bv)
        pc.constraint_region.primitive_poses.append(bv_pose)
        pc.weight = 1.0
        goal_constraints.position_constraints.append(pc)

        # Orientation constraint
        oc = OrientationConstraint()
        oc.header.frame_id = self.BASE_LINK
        oc.link_name = self.EEF_LINK
        oc.orientation = pose_msg.orientation
        oc.absolute_x_axis_tolerance = 0.2
        oc.absolute_y_axis_tolerance = 0.2
        oc.absolute_z_axis_tolerance = 3.14
        oc.weight = 1.0
        goal_constraints.orientation_constraints.append(oc)

        mp_request.goal_constraints.append(goal_constraints)

        # Set start state
        if self._current_joints is not None:
            mp_request.start_state.joint_state = self._current_joints

        # Call planning service
        future = self._plan_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)

        if future.result() is None:
            self.logger.error("Planning failed: no result")
            return False

        result = future.result()
        if result.motion_plan_response.error_code.val != MoveItErrorCodes.SUCCESS:
            self.logger.error(
                f"Planning failed with code: {result.motion_plan_response.error_code.val}"
            )
            return False

        return self._execute_trajectory(result.motion_plan_response.trajectory)

    def _execute_trajectory(self, trajectory):
        """
        Execute a planned trajectory via the ExecuteTrajectory action.

        Args:
            trajectory: moveit_msgs/RobotTrajectory (the planned path)

        Returns:
            success: True if execution completed
        """
        if not self._execute_client.wait_for_server(timeout_sec=5.0):
            self.logger.error("ExecuteTrajectory action server not available")
            return False

        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory

        self.logger.info(
            f"Executing trajectory with {len(trajectory.joint_trajectory.points)} points"
        )

        send_goal_future = self._execute_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send_goal_future, timeout_sec=5.0)

        goal_handle = send_goal_future.result()
        if not goal_handle or not goal_handle.accepted:
            self.logger.error("Trajectory execution goal rejected")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=60.0)

        result = result_future.result()
        if result and result.result.error_code.val == MoveItErrorCodes.SUCCESS:
            self.logger.info("Trajectory execution succeeded")
            return True
        else:
            code = result.result.error_code.val if result else 'timeout'
            self.logger.error(f"Trajectory execution failed: {code}")
            return False

    def execute_targets(self, targets, motion_delay=0.1, velocity_scaling=0.2):
        """
        Execute a sequence of target poses.

        Args:
            targets: list of [x,y,z,qx,qy,qz,qw] poses
            motion_delay: delay between targets (seconds)
            velocity_scaling: velocity scaling for each motion

        Returns:
            success_count: number of successfully reached targets
            total: total number of targets
        """
        success_count = 0
        total = len(targets)

        self.logger.info(f"Executing {total} target poses...")

        for i, target in enumerate(targets):
            self.logger.info(f"Target {i + 1}/{total}")
            success = self.go_to_pose_goal(
                target,
                velocity_scaling=velocity_scaling,
                acceleration_scaling=velocity_scaling
            )
            if success:
                success_count += 1
            else:
                self.logger.warn(f"Failed to reach target {i + 1}, continuing...")

            if motion_delay > 0:
                time.sleep(motion_delay)

        self.logger.info(f"Completed: {success_count}/{total} targets reached")
        return success_count, total

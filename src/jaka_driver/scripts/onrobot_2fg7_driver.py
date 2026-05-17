#!/usr/bin/env python3

import time
import urllib.request
import urllib.error
from typing import Optional

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node

from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool


class OnRobot2FG7Driver(Node):
    def __init__(self) -> None:
        super().__init__("onrobot_2fg7_driver")

        self.declare_parameter("gripper_ip", "192.168.1.100")
        self.declare_parameter("gripper_port", 502)  # kept for compatibility
        self.declare_parameter("unit_id", 65)         # kept for compatibility
        self.declare_parameter("connect_timeout_sec", 2.0)

        self.declare_parameter("action_name", "/gripper_controller/follow_joint_trajectory")
        self.declare_parameter("joint_name", "left_finger_joint")

        self.declare_parameter("min_position_m", 0.0)
        self.declare_parameter("max_position_m", 0.019)

        # OnRobot 2FG7 via Compute Box REST API parameters
        self.declare_parameter("device_id", 0)
        self.declare_parameter("grip_mode", "external")   # "external" or "internal"
        self.declare_parameter("default_force_n", 40)      # Newtons (20-140)
        self.declare_parameter("default_speed_pct", 50)    # Percent (10-100)
        self.declare_parameter("min_width_mm", 0)          # mm
        self.declare_parameter("max_width_mm", 37)         # mm (external grip max)
        self.declare_parameter("http_port", 80)            # Compute Box web port

        self.declare_parameter("open_position_m", 0.0)
        self.declare_parameter("close_position_m", 0.019)

        self.gripper_ip = self.get_parameter("gripper_ip").value
        self.connect_timeout_sec = float(self.get_parameter("connect_timeout_sec").value)

        self.action_name = self.get_parameter("action_name").value
        self.joint_name = self.get_parameter("joint_name").value

        self.min_position_m = float(self.get_parameter("min_position_m").value)
        self.max_position_m = float(self.get_parameter("max_position_m").value)

        self.device_id = int(self.get_parameter("device_id").value)
        self.grip_mode = self.get_parameter("grip_mode").value
        self.default_force_n = int(self.get_parameter("default_force_n").value)
        self.default_speed_pct = int(self.get_parameter("default_speed_pct").value)
        self.min_width_mm = int(self.get_parameter("min_width_mm").value)
        self.max_width_mm = int(self.get_parameter("max_width_mm").value)
        self.http_port = int(self.get_parameter("http_port").value)

        self.open_position_m = float(self.get_parameter("open_position_m").value)
        self.close_position_m = float(self.get_parameter("close_position_m").value)

        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            self.action_name,
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        self._set_gripper_service = self.create_service(
            SetBool,
            "~/set_gripper",
            self.set_gripper_callback,
        )

        self._base_url = f"http://{self.gripper_ip}:{self.http_port}"

        # Track current position and publish to /joint_states so MoveIt knows
        self._current_position_m = 0.0
        self._joint_state_pub = self.create_publisher(JointState, "/joint_states", 10)
        self._joint_state_timer = self.create_timer(0.1, self._publish_joint_state)

        self.get_logger().info(
            "OnRobot 2FG7 REST driver ready: "
            f"{self._base_url}, device_id={self.device_id}, "
            f"mode={self.grip_mode}, force={self.default_force_n}N, "
            f"speed={self.default_speed_pct}%"
        )

    def goal_callback(self, goal_request: FollowJointTrajectory.Goal) -> GoalResponse:
        if not goal_request.trajectory.points:
            self.get_logger().error("Rejecting goal: empty trajectory")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, _goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        result = FollowJointTrajectory.Result()

        try:
            trajectory = goal_handle.request.trajectory
            joint_index = self._resolve_joint_index(trajectory.joint_names)

            if not trajectory.points:
                goal_handle.abort()
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                result.error_string = "Empty trajectory"
                return result

            # Send only the final target position — the Compute Box REST API
            # expects a single goal command, not rapid intermediate waypoints.
            final_point = trajectory.points[-1]
            target_position = final_point.positions[joint_index]
            self._command_gripper_position(target_position)

            # Wait for the gripper to physically reach the target
            total_time_sec = (
                float(final_point.time_from_start.sec)
                + float(final_point.time_from_start.nanosec) * 1e-9
            )
            if total_time_sec > 0.0:
                time.sleep(total_time_sec)

            goal_handle.succeed()
            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            result.error_string = "Gripper trajectory executed"
            return result

        except Exception as error:
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = str(error)
            self.get_logger().error(f"Gripper trajectory failed: {error}")
            return result

    def set_gripper_callback(self, request: SetBool.Request, response: SetBool.Response):
        try:
            target_position = self.open_position_m if request.data else self.close_position_m
            self._command_gripper_position(target_position)
            response.success = True
            response.message = (
                f"Gripper moved to {'open' if request.data else 'close'} "
                f"({target_position:.6f} m)"
            )
        except Exception as error:
            response.success = False
            response.message = str(error)
            self.get_logger().error(f"Set gripper failed: {error}")
        return response

    def _resolve_joint_index(self, joint_names) -> int:
        if self.joint_name in joint_names:
            return joint_names.index(self.joint_name)
        if len(joint_names) == 1:
            self.get_logger().warn(
                f"Joint {self.joint_name} not found, using first joint {joint_names[0]}"
            )
            return 0
        raise ValueError(
            f"Trajectory must include joint '{self.joint_name}'. Got joints: {list(joint_names)}"
        )

    def _position_m_to_width_mm(self, position_m: float) -> int:
        """Convert joint position in meters to gripper width in mm."""
        if self.max_position_m <= self.min_position_m:
            raise ValueError("Invalid position limits: max_position_m must be > min_position_m")

        clamped = min(max(position_m, self.min_position_m), self.max_position_m)
        ratio = 1.0 - (clamped - self.min_position_m) / (self.max_position_m - self.min_position_m)
        width_mm = int(round(
            self.min_width_mm + ratio * (self.max_width_mm - self.min_width_mm)
        ))
        return width_mm

    def _command_gripper_position(self, position_m: float) -> None:
        width_mm = self._position_m_to_width_mm(position_m)

        # OnRobot Compute Box REST API:
        #   GET /api/dc/twofg/grip_external/{id}/{width}/{force}/{speed}
        #   GET /api/dc/twofg/grip_internal/{id}/{width}/{force}/{speed}
        #   GET /api/dc/twofg/stop/{id}
        endpoint = f"grip_{self.grip_mode}"
        url = (
            f"{self._base_url}/api/dc/twofg/{endpoint}/"
            f"{self.device_id}/{width_mm}/{self.default_force_n}/{self.default_speed_pct}"
        )

        try:
            req = urllib.request.Request(url)
            with urllib.request.urlopen(req, timeout=self.connect_timeout_sec) as resp:
                resp_body = resp.read().decode().strip()
                if resp_body != "0":
                    raise RuntimeError(
                        f"Compute Box returned error: {resp_body}"
                    )
        except urllib.error.URLError as e:
            raise RuntimeError(
                f"Could not connect to Compute Box at {self._base_url}: {e}"
            )

        self._current_position_m = position_m

        self.get_logger().info(
            f"Commanded gripper: position_m={position_m:.6f}, "
            f"width_mm={width_mm}, force={self.default_force_n}N, "
            f"speed={self.default_speed_pct}%"
        )

    def _publish_joint_state(self) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [self.joint_name]
        msg.position = [self._current_position_m]
        self._joint_state_pub.publish(msg)

    def stop_gripper(self) -> None:
        """Send stop command to gripper."""
        url = f"{self._base_url}/api/dc/twofg/stop/{self.device_id}"
        try:
            req = urllib.request.Request(url)
            with urllib.request.urlopen(req, timeout=self.connect_timeout_sec) as resp:
                resp.read()
        except urllib.error.URLError as e:
            self.get_logger().error(f"Failed to stop gripper: {e}")


def main() -> None:
    rclpy.init()
    node = OnRobot2FG7Driver()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

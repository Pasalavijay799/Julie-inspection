#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
iit_logo_stable.py — JAKA ZU5 IIT Logo Pick-and-Place
Joint positions sourced from all_joint_angles.xlsx (Sheet1).

Arm    : /jaka_zu5_controller/follow_joint_trajectory
Gripper: /gripper_controller/follow_joint_trajectory
Safety : /safety_override  (Bool — pause/resume, checked between moves)

MOTION STRATEGY:
  The robot STOPS at every named waypoint before moving to the next one.
  This matches the Excel sequence exactly: each row is a discrete stop point.
  Gripper open/close commands are inserted at the exact positions indicated
  by the 'grip' rows in the spreadsheet.

Run after:
  ros2 launch jaka_visual_inspection simulation.launch.py
  ros2 run jaka_planner iit_logo_stable.py --ros-args -p use_sim_time:=true

Or on real robot:
  ros2 launch jaka_planner moveit_server.launch.py ip:=192.168.0.50 model:=zu5 \\
      use_gripper:=true gripper_ip:=192.168.0.75 use_camera:=true
  ros2 launch jaka_zu5_moveit_config demo.launch.py use_rviz_sim:=false
  ros2 run jaka_planner iit_logo_stable.py
"""

import time
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Bool
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState

# Match moveit_server's BEST_EFFORT publisher QoS
_QOS_BEST_EFFORT = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)

JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

# Motion timing: seconds per waypoint segment (stop-and-move)
MOVE_DURATION = 5.0   # seconds to travel to each waypoint
SETTLE_PAUSE  = 0.3   # seconds to settle after each move before next


def _to_rad(deg_list):
    return [math.radians(d) for d in deg_list]


class IITLogoPickAndPlace(Node):
    def __init__(self):
        super().__init__('iit_logo_pick_and_place')

        self.arm_client = ActionClient(
            self, FollowJointTrajectory,
            '/jaka_zu5_controller/follow_joint_trajectory'
        )
        self.gripper_client = ActionClient(
            self, FollowJointTrajectory,
            '/gripper_controller/follow_joint_trajectory'
        )

        # Joint-state tracking — BEST_EFFORT to match moveit_server publisher
        self.current_positions = None
        self.create_subscription(JointState, '/joint_states', self._joint_cb, _QOS_BEST_EFFORT)

        # Safety pause — checked between every move
        self.pause_flag = False
        self.create_subscription(Bool, '/safety_override', self._safety_cb, 10)

        self.get_logger().info(
            'IIT Logo node ready. Arm: /jaka_zu5_controller/follow_joint_trajectory'
        )

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def _joint_cb(self, msg: JointState):
        if set(JOINT_NAMES).issubset(msg.name):
            self.current_positions = [
                msg.position[msg.name.index(j)] for j in JOINT_NAMES
            ]

    def _safety_cb(self, msg: Bool):
        if msg.data and not self.pause_flag:
            self.get_logger().warn('HUMAN NEAR — PAUSING motion')
            self.pause_flag = True
        elif not msg.data and self.pause_flag:
            self.get_logger().info('SAFE — RESUMING motion')
            self.pause_flag = False

    # ------------------------------------------------------------------
    # Startup
    # ------------------------------------------------------------------
    def wait_for_servers(self) -> bool:
        self.get_logger().info('Waiting for action servers...')
        self.arm_client.wait_for_server()
        if not self.gripper_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn('Gripper server not found — gripper moves will be skipped.')
        else:
            self.get_logger().info('Gripper server ready.')

        self.get_logger().info('Waiting for /joint_states synchronization...')
        while rclpy.ok() and self.current_positions is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        time.sleep(1.0)
        self.get_logger().info('All servers ready — starting routine.')
        return True

    def _check_pause(self):
        """Spin briefly to process safety callbacks, then wait if paused."""
        rclpy.spin_once(self, timeout_sec=0.05)
        while self.pause_flag and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

    # ------------------------------------------------------------------
    # Arm motion — STOP-AND-MOVE at every waypoint
    # ------------------------------------------------------------------
    def move_to(self, target_rad: list, label: str = '', duration: float = MOVE_DURATION):
        """
        Move the arm to a single target joint configuration and STOP there.
        The robot fully reaches this waypoint before any further command is sent.

        Linearly interpolates at INTERP_DT = 0.1 s intervals so that
        step_num per servo_j call stays small (≈12), which the JAKA SDK accepts.
        """
        self._check_pause()

        INTERP_DT = 0.1   # 0.1 s → step_num = int(0.1/0.008) = 12  ✓

        rclpy.spin_once(self, timeout_sec=0.1)
        start = list(self.current_positions)

        # Build interpolated trajectory: start → target
        n_steps = max(1, int(round(duration / INTERP_DT)))

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINT_NAMES
        goal.trajectory.header.stamp.sec = 0
        goal.trajectory.header.stamp.nanosec = 0

        for k in range(n_steps + 1):
            alpha = k / n_steps
            t     = alpha * duration
            pos   = [start[j] + alpha * (target_rad[j] - start[j]) for j in range(6)]

            pt = JointTrajectoryPoint()
            pt.positions     = pos
            pt.velocities    = [0.0] * 6
            pt.accelerations = [0.0] * 6
            sec     = int(math.floor(t))
            nanosec = int((t - sec) * 1e9)
            pt.time_from_start = Duration(sec=sec, nanosec=nanosec)
            goal.trajectory.points.append(pt)

        tag = f' [{label}]' if label else ''
        self.get_logger().info(
            f'→ Moving to{tag} — {n_steps + 1} interpolated pts over {duration:.1f}s'
        )

        future = self.arm_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error(f'Arm goal REJECTED for{tag}!')
            return

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        # Update internal position to the target so the next segment starts correctly
        self.current_positions = list(target_rad)
        time.sleep(SETTLE_PAUSE)
        self.get_logger().info(f'✓ Reached{tag}')

    # ------------------------------------------------------------------
    # Gripper motion
    # ------------------------------------------------------------------
    def move_gripper(self, width_m: float, duration_sec: float = 2.0):
        """
        Command the OnRobot 2FG7 gripper.

        Driver mapping:
          position_m = 0.0   → width = 37 mm  (OPEN)
          position_m = 0.019 → width =  0 mm  (CLOSED)
        """
        if not self.gripper_client.server_is_ready():
            self.get_logger().warn('Gripper not available — skipping.')
            return

        self._check_pause()
        label = 'OPEN' if width_m < 0.01 else 'CLOSE'
        self.get_logger().info(f'Gripper {label} (position={width_m:.4f} m)')

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['left_finger_joint']
        goal.trajectory.header.stamp.sec = 0
        goal.trajectory.header.stamp.nanosec = 0

        # Point 0 — start at t=0
        p0 = JointTrajectoryPoint()
        p0.positions = [width_m]
        p0.velocities = [0.0]
        p0.time_from_start = Duration(sec=0, nanosec=0)
        goal.trajectory.points.append(p0)

        # Point 1 — target at t=duration_sec
        p1 = JointTrajectoryPoint()
        p1.positions = [width_m]
        p1.velocities = [0.0]
        sec     = int(math.floor(duration_sec))
        nanosec = int((duration_sec - sec) * 1e9)
        p1.time_from_start = Duration(sec=sec, nanosec=nanosec)
        goal.trajectory.points.append(p1)

        future = self.gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        # Sleep for physical motion — don't block on result_future to avoid
        # stale-response warnings on the next send_goal_async call.
        time.sleep(duration_sec + 0.5)


# --------------------------------------------------------------------------
# Gripper constants  (driver mapping: position_m=0.0 → 37mm OPEN,
#                                     position_m=0.019 → 0mm CLOSED)
# --------------------------------------------------------------------------
GRIPPER_OPEN  = 0.0    # 37 mm — wide open
GRIPPER_CLOSE = 0.019  # 0 mm  — fully closed


# --------------------------------------------------------------------------
# IIT Logo sequence — ALL waypoints from all_joint_angles.xlsx (Sheet1)
# Robot STOPS at every named point before continuing to the next.
# Gripper commands inserted at the exact rows where 'grip' appears in the sheet.
# --------------------------------------------------------------------------

def execute_logo_routine(node: IITLogoPickAndPlace):
    if not node.wait_for_servers():
        return

    # ── Joint positions (degrees) from all_joint_angles.xlsx ─────────────────
    # NOTE: J5 values in the sheet are ~270° which is equivalent to -90° in the
    # robot's ±180° joint range.  Convert: 270° - 360° = -90°.
    # J6 values that are large negative (e.g. -219°) are also wrapped to ±180°.
    # All angles passed to _to_rad() below are already in the sheet's raw degrees;
    # the JAKA SDK / moveit_server accepts values outside ±180° if joints allow,
    # so we use the raw values exactly as recorded.

    p_home      = _to_rad([  0.000,  90.000,   0.000,  90.000, 180.000,   0.000])

    # ── Pick-1 sequence ────────────────────────────────────────────────────────
    p_target1   = _to_rad([  3.732, 126.699,  59.307,  83.978, 270.473, -42.560])
    p_grip1     = _to_rad([  3.732, 128.211,  66.886,  74.567, 270.473, -42.560])

    # ── Carry-1 → Place-1 sequence ────────────────────────────────────────────
    p_target2   = _to_rad([  3.732, 126.480,  46.999,  96.186, 270.473, -42.560])
    p_target3   = _to_rad([  5.186,  96.291,  89.918,  83.443, 270.464, -41.106])
    p_target4   = _to_rad([ 52.168, 125.084,  49.232,  95.738, 270.577, -83.266])
    p_target5   = _to_rad([ 95.302, 125.189,  49.182,  95.273, 270.458, -40.130])
    p_target6   = _to_rad([ 96.125, 132.070,  74.538,  63.030, 270.453, -39.307])

    # ── Retract after Place-1 → Pick-2 approach ───────────────────────────────
    p_target7   = _to_rad([ 96.142, 128.666,  31.632, 109.341, 270.453, -39.290])
    p_target9   = _to_rad([100.391,  82.688,  92.114,  94.804, 270.425, -35.041])
    p_target10  = _to_rad([ 45.475, 124.631,  38.690, 106.118, 270.144,  -2.385])
    p_target11  = _to_rad([  5.487, 102.207,  71.616,  95.805, 270.444, -37.904])
    p_target12  = _to_rad([  4.236, 131.974,  25.647, 112.016, 270.452, -39.155])
    p_target13  = _to_rad([  3.735, 127.613,  65.131,  76.897, 270.455, -39.656])
    p_target14  = _to_rad([  3.735, 130.870,  71.614,  67.157, 270.455, -39.656])

    # ── Carry-2 → Place-2 sequence ────────────────────────────────────────────
    p_second1   = _to_rad([  3.735, 126.288,  49.228,  94.125, 270.455, -39.656])
    p_second2   = _to_rad([  4.423,  85.842, 100.583,  83.211, 270.450, -38.968])
    p_second3   = _to_rad([ 61.975, 130.501,  40.085,  98.839, 269.934,  18.582])
    p_second4   = _to_rad([ 84.636, 106.898,  76.763,  86.091, 270.487, -49.197])
    p_second5   = _to_rad([ 84.835, 126.550,  46.829,  96.371, 270.486, -48.998])
    p_second6   = _to_rad([ 86.908, 134.263,  71.972,  63.505, 270.480, -47.495])

    # ── Retract after Place-2 → Pick-3 approach ───────────────────────────────
    p_third1    = _to_rad([ 86.908, 127.839,  46.806,  95.094, 270.480, -47.495])
    p_third2    = _to_rad([ 85.053,  88.761,  99.966,  81.027, 270.488, -49.351])
    p_third3    = _to_rad([ 47.147, 125.164,  51.230,  93.712, 270.536, -87.257])
    p_third4    = _to_rad([ 10.261,  99.830,  87.931,  82.611, 269.599, -213.200])
    p_third5    = _to_rad([  7.482, 131.199,  40.944,  98.208, 269.582, -215.980])
    p_third6    = _to_rad([  3.800, 132.856,  64.406,  73.062, 269.560, -219.661])
    p_pickT     = _to_rad([  3.800, 137.327,  69.716,  63.281, 269.560, -219.661])

    # ── Carry-3 → Place-3 sequence ────────────────────────────────────────────
    p_pickup1   = _to_rad([  3.800, 130.027,  46.488,  93.809, 269.560, -219.661])
    p_pickup2   = _to_rad([  5.354,  98.166,  93.415,  78.755, 269.569, -218.107])
    p_pickup3   = _to_rad([ 55.486, 137.570,  32.744, 100.232, 269.981, -167.977])
    p_pickup4   = _to_rad([ 70.709, 113.121,  73.763,  83.258, 269.473, -240.869])
    p_pickup6   = _to_rad([ 75.129, 143.812,  20.841, 105.529, 269.485, -236.448])
    p_pickup7   = _to_rad([ 78.421, 139.311,  49.086,  81.779, 269.483, -237.186])
    p_pickup8   = _to_rad([ 78.758, 141.894,  58.083,  70.202, 269.484, -236.849])

    # ── Final park ────────────────────────────────────────────────────────────
    p_uplast    = _to_rad([ 86.377, 142.558,  18.014, 109.671, 269.517, -229.230])

    # ── End / resting home position ───────────────────────────────────────────
    p_end_home  = _to_rad([  0.000,   0.344, 146.804, 114.819, 100.246,   0.000])

    # ==========================================================================
    # SEQUENCE EXECUTION — stop-and-move at every named waypoint
    # ==========================================================================
    node.get_logger().info('=== IIT Logo Pick-and-Place Routine START ===')

    # ── Step 0: Open gripper, move to home ────────────────────────────────────
    node.move_gripper(GRIPPER_OPEN)
    node.move_to(p_home, label='home')

    # ── Step 1: Approach pick-1 (gripper open) ────────────────────────────────
    node.move_to(p_target1, label='target_1')
    node.move_to(p_grip1,   label='grip_approach_1')

    # ── Gripper CLOSE — grip object 1 ─────────────────────────────────────────
    node.move_gripper(GRIPPER_CLOSE)

    # ── Step 2: Carry object 1 → Place-1 ─────────────────────────────────────
    node.move_to(p_target2,  label='target2')
    node.move_to(p_target3,  label='target3')
    node.move_to(p_target4,  label='target4')
    node.move_to(p_target5,  label='target5')
    node.move_to(p_target6,  label='target6')   # place position 1

    # ── Gripper OPEN — release object 1 ───────────────────────────────────────
    node.move_gripper(GRIPPER_OPEN)

    # ── Step 3: Retract from Place-1 → Approach pick-2 ───────────────────────
    node.move_to(p_target7,  label='target7')
    node.move_to(p_target9,  label='target9')
    node.move_to(p_target10, label='target10')
    node.move_to(p_target11, label='target11')
    node.move_to(p_target12, label='target12')
    node.move_to(p_target13, label='target13')
    node.move_to(p_target14, label='target14')  # pick position 2

    # ── Gripper CLOSE — grip object 2 ─────────────────────────────────────────
    node.move_gripper(GRIPPER_CLOSE)

    # ── Step 4: Carry object 2 → Place-2 ─────────────────────────────────────
    node.move_to(p_second1, label='second_i')
    node.move_to(p_second2, label='secondi2')
    node.move_to(p_second3, label='second3')
    node.move_to(p_second4, label='second4')
    node.move_to(p_second5, label='second5')
    node.move_to(p_second6, label='second6')    # place position 2

    # ── Gripper OPEN — release object 2 ───────────────────────────────────────
    node.move_gripper(GRIPPER_OPEN)

    # ── Step 5: Retract from Place-2 → Approach pick-3 ───────────────────────
    node.move_to(p_third1, label='third1')
    node.move_to(p_third2, label='third2')
    node.move_to(p_third3, label='third3')
    node.move_to(p_third4, label='third4')
    node.move_to(p_third5, label='third5')
    node.move_to(p_third6, label='third6')
    node.move_to(p_pickT,  label='pickT')       # pick position 3

    # ── Gripper CLOSE — grip object 3 ─────────────────────────────────────────
    node.move_gripper(GRIPPER_CLOSE)

    # ── Step 6: Carry object 3 → Place-3 ─────────────────────────────────────
    node.move_to(p_pickup1, label='pick_upT')
    node.move_to(p_pickup2, label='pick_upT2')
    node.move_to(p_pickup3, label='pick_up3')
    node.move_to(p_pickup4, label='pick_upT4')
    node.move_to(p_pickup6, label='pick_upT6')
    node.move_to(p_pickup7, label='pickupT7')
    node.move_to(p_pickup8, label='pickupT8')   # place position 3

    # ── Gripper OPEN — release object 3 ───────────────────────────────────────
    node.move_gripper(GRIPPER_OPEN)

    # ── Step 7: Park to final position ────────────────────────────────────────
    node.move_to(p_uplast, label='uplast')

    # ── Step 8: Return to end/resting home ───────────────────────────────────
    node.move_to(p_end_home, label='end_home')

    node.get_logger().info('=== IIT Logo Routine COMPLETE ===')


# --------------------------------------------------------------------------
# Entry point
# --------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = IITLogoPickAndPlace()
    try:
        execute_logo_routine(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

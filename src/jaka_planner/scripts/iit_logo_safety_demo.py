#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
iit_logo_stable.py — JAKA ZU5 IIT Logo Pick-and-Place (Looping)

Part 1 : waypoints from all_joint_angles.xlsx
Part 2 : waypoints from all_40_joint_angles.xlsx

Execution loop:
  [Part 1] → [Part 2] → [Part 1] → [Part 2] → … until Ctrl+C

At startup the user is asked:
  • "Conveyer? (y/n)"
      y → start from waypoint 1 of Part 1 (beginning)
      n → "Enter start waypoint name:" → start from that waypoint in the
           combined sequence and loop from the beginning on every subsequent cycle

Press Ctrl+C at any time to stop gracefully after the current waypoint.

Run (simulation):
  ros2 launch jaka_visual_inspection simulation.launch.py
  ros2 run jaka_planner iit_logo_stable.py --ros-args -p use_sim_time:=true

Run (real robot):
  ros2 launch jaka_planner moveit_server.launch.py ip:=192.168.0.50 model:=zu5 \\
      use_gripper:=true gripper_ip:=192.168.0.75 use_camera:=false
  ros2 run jaka_planner iit_logo_stable.py
"""

import sys
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

# ── QoS ──────────────────────────────────────────────────────────────────────
_QOS_BEST_EFFORT = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)

JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

# Motion timing
MOVE_DURATION = 5.0   # seconds per waypoint
SETTLE_PAUSE  = 0.3   # seconds to settle after each move


def _to_rad(deg_list):
    return [math.radians(d) for d in deg_list]


# =============================================================================
# Node
# =============================================================================

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

        self.current_positions = None
        self.create_subscription(JointState, '/joint_states', self._joint_cb, _QOS_BEST_EFFORT)

        self.pause_flag  = False
        self.stop_flag   = False
        self.active_goal_handle         = None
        self.active_gripper_goal_handle = None   # track gripper separately
        self.current_gripper_pos        = GRIPPER_OPEN  # last commanded gripper
        self.create_subscription(Bool, '/safety_override', self._safety_cb, 10)

        self.get_logger().info('IIT Logo node ready.')

    # ------------------------------------------------------------------
    def _joint_cb(self, msg: JointState):
        if set(JOINT_NAMES).issubset(msg.name):
            self.current_positions = [
                msg.position[msg.name.index(j)] for j in JOINT_NAMES
            ]

    def _safety_cb(self, msg: Bool):
        if msg.data and not self.pause_flag:
            self.get_logger().warn('HUMAN NEAR — PAUSING')
            self.pause_flag = True
            # Cancel active arm goal
            if self.active_goal_handle is not None:
                self.active_goal_handle.cancel_goal_async()
            # Cancel active gripper goal (if any)
            if self.active_gripper_goal_handle is not None:
                self.active_gripper_goal_handle.cancel_goal_async()
        elif not msg.data and self.pause_flag:
            self.get_logger().info('SAFE — RESUMING')
            self.pause_flag = False

    # ------------------------------------------------------------------
    def wait_for_servers(self) -> bool:
        self.get_logger().info('Waiting for action servers…')
        self.arm_client.wait_for_server()
        if not self.gripper_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn('Gripper server not found — gripper moves skipped.')
        else:
            self.get_logger().info('Gripper server ready.')
        self.get_logger().info('Waiting for /joint_states…')
        while rclpy.ok() and self.current_positions is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        time.sleep(1.0)
        self.get_logger().info('All servers ready.')
        return True

    def _check_pause(self):
        rclpy.spin_once(self, timeout_sec=0.05)
        while self.pause_flag and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

    # ------------------------------------------------------------------
    # Arm — stop-and-move at every single waypoint with resume capability
    # ------------------------------------------------------------------
    def move_to(self, target_rad: list, label: str = '', duration: float = MOVE_DURATION):
        if self.stop_flag:
            return

        tag = f' [{label}]' if label else ''
        self.get_logger().info(f'→ Moving to{tag}')

        while rclpy.ok() and not self.stop_flag:
            self._check_pause()

            rclpy.spin_once(self, timeout_sec=0.1)
            start = list(self.current_positions)
            
            # Check if we are already there (or very close)
            max_dist = max(abs(target_rad[j] - start[j]) for j in range(6))
            if max_dist < 0.02:
                self.current_positions = list(target_rad)
                time.sleep(SETTLE_PAUSE)
                self.get_logger().info(f'✓ Reached{tag}')
                return

            # Compute remaining duration (use generous minimum for smooth resume)
            speed        = 0.5
            rem_duration = max(2.0, max_dist / speed)   # ←4 minimum 2 s — prevents harsh jerk

            INTERP_DT = 0.1
            n_steps = max(1, int(round(rem_duration / INTERP_DT)))

            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = JOINT_NAMES
            goal.trajectory.header.stamp.sec = 0
            goal.trajectory.header.stamp.nanosec = 0

            for k in range(n_steps + 1):
                alpha = k / n_steps
                t = alpha * rem_duration
                pos = [start[j] + alpha * (target_rad[j] - start[j]) for j in range(6)]
                pt = JointTrajectoryPoint()
                pt.positions = pos
                pt.velocities = [0.0] * 6
                pt.accelerations = [0.0] * 6
                sec = int(math.floor(t))
                nanosec = int((t - sec) * 1e9)
                pt.time_from_start = Duration(sec=sec, nanosec=nanosec)
                goal.trajectory.points.append(pt)

            future = self.arm_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, future)
            self.active_goal_handle = future.result()
            
            if not self.active_goal_handle or not self.active_goal_handle.accepted:
                self.get_logger().error(f'Goal REJECTED{tag}')
                return

            result_future = self.active_goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)
            self.active_goal_handle = None

            res = result_future.result().result
            if res.error_code == 0:
                self.current_positions = list(target_rad)
                time.sleep(SETTLE_PAUSE)
                self.get_logger().info(f'✓ Reached{tag}')
                return
            else:
                # Goal was cancelled (safety pause) — wait for robot to fully stop
                # before re-planning.  Reading current_positions too early gives a
                # mid-motion joint state and causes the harsh jerk on resume.
                self.get_logger().warn(f'Goal interrupted{tag} — waiting for robot to settle…')
                time.sleep(1.0)   # physical deceleration time
                for _ in range(10):
                    rclpy.spin_once(self, timeout_sec=0.05)
                self.get_logger().warn(f'Re-planning{tag} from settled position…')

    # ------------------------------------------------------------------
    # Gripper
    # ------------------------------------------------------------------
    def move_gripper(self, width_m: float, duration_sec: float = 2.0):
        if self.stop_flag:
            return
        if not self.gripper_client.server_is_ready():
            self.get_logger().warn('Gripper not available — skipping.')
            return
        self._check_pause()
        label = 'OPEN' if width_m < 0.01 else 'CLOSE'
        self.get_logger().info(f'Gripper {label}')

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['left_finger_joint']
        goal.trajectory.header.stamp.sec = 0
        goal.trajectory.header.stamp.nanosec = 0

        p0 = JointTrajectoryPoint()
        p0.positions = [width_m]; p0.velocities = [0.0]
        p0.time_from_start = Duration(sec=0, nanosec=0)
        goal.trajectory.points.append(p0)

        p1 = JointTrajectoryPoint()
        p1.positions = [width_m]; p1.velocities = [0.0]
        sec = int(math.floor(duration_sec))
        p1.time_from_start = Duration(sec=sec, nanosec=int((duration_sec - sec) * 1e9))
        goal.trajectory.points.append(p1)

        future = self.gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        gh = future.result()
        if gh and gh.accepted:
            self.active_gripper_goal_handle = gh
            self.current_gripper_pos        = width_m   # save for resume
            result_f = gh.get_result_async()
            rclpy.spin_until_future_complete(self, result_f,
                                             timeout_sec=duration_sec + 2.0)
            self.active_gripper_goal_handle = None
        else:
            time.sleep(duration_sec + 0.5)


# =============================================================================
# Gripper constants
# =============================================================================
GRIPPER_OPEN  = 0.0    # 37 mm wide open
GRIPPER_CLOSE = 0.019  # 0 mm fully closed


# =============================================================================
# PART 1 — all_joint_angles.xlsx
# Each entry: ('label', [J1..J6]) or ('grip', 'OPEN'/'CLOSE')
# =============================================================================
def _build_part1():
    return [
        ('grip',        'OPEN'),
        ('home',        _to_rad([  0.000,  90.000,   0.000,  90.000, 180.000,   0.000])),
        ('target_1',    _to_rad([  3.732, 126.699,  59.307,  83.978, 270.473, -42.560])),
        ('grip_app1',   _to_rad([  3.732, 128.211,  66.886,  74.567, 270.473, -42.560])),
        ('grip',        'CLOSE'),
        ('target2',     _to_rad([  3.732, 126.480,  46.999,  96.186, 270.473, -42.560])),
        ('target3',     _to_rad([  5.186,  96.291,  89.918,  83.443, 270.464, -41.106])),
        ('target4',     _to_rad([ 52.168, 125.084,  49.232,  95.738, 270.577, -83.266])),
        ('target5',     _to_rad([ 95.302, 125.189,  49.182,  95.273, 270.458, -40.130])),
        ('target6',     _to_rad([ 95.852, 133.039,  75.315,  61.286, 270.454, -39.580])),
        ('grip',        'OPEN'),
        ('target7',     _to_rad([ 96.142, 128.666,  31.632, 109.341, 270.453, -39.290])),
        ('target9',     _to_rad([100.391,  82.688,  92.114,  94.804, 270.425, -35.041])),
        ('target10',    _to_rad([ 45.475, 124.631,  38.690, 106.118, 270.144,  -2.385])),
        ('target11',    _to_rad([  5.487, 102.207,  71.616,  95.805, 270.444, -37.904])),
        ('target12',    _to_rad([  4.236, 131.974,  25.647, 112.016, 270.452, -39.155])),
        ('target13',    _to_rad([  3.735, 127.613,  65.131,  76.897, 270.455, -39.656])),
        ('target14',    _to_rad([  3.735, 130.870,  71.614,  67.157, 270.455, -39.656])),
        ('grip',        'CLOSE'),
        ('second_i',    _to_rad([  3.735, 126.288,  49.228,  94.125, 270.455, -39.656])),
        ('secondi2',    _to_rad([  4.423,  85.842, 100.583,  83.211, 270.450, -38.968])),
        ('second3',     _to_rad([ 61.975, 130.501,  40.085,  98.839, 269.934,  18.582])),
        ('second4',     _to_rad([ 84.636, 106.898,  76.763,  86.091, 270.487, -49.197])),
        ('second5',     _to_rad([ 84.835, 126.550,  46.829,  96.371, 270.486, -48.998])),
        ('second6',     _to_rad([ 86.908, 134.263,  71.972,  63.505, 270.480, -47.495])),
        ('grip',        'OPEN'),
        ('third1',      _to_rad([ 86.908, 127.839,  46.806,  95.094, 270.480, -47.495])),
        ('third2',      _to_rad([ 85.053,  88.761,  99.966,  81.027, 270.488, -49.351])),
        ('third3',      _to_rad([ 47.147, 125.164,  51.230,  93.712, 270.536, -87.257])),
        ('third4',      _to_rad([ 10.261,  99.830,  87.931,  82.611, 269.599, -213.200])),
        ('third5',      _to_rad([  7.482, 131.199,  40.944,  98.208, 269.582, -215.980])),
        ('third6',      _to_rad([  3.800, 132.856,  64.406,  73.062, 269.560, -219.661])),
        ('pickT',       _to_rad([  3.800, 137.327,  69.716,  63.281, 269.560, -219.661])),
        ('grip',        'CLOSE'),
        ('pick_upT',    _to_rad([  3.800, 130.027,  46.488,  93.809, 269.560, -219.661])),
        ('pick_upT2',   _to_rad([  5.354,  98.166,  93.415,  78.755, 269.569, -218.107])),
        ('pick_up3',    _to_rad([ 55.486, 137.570,  32.744, 100.232, 269.981, -167.977])),
        ('pick_upT4',   _to_rad([ 70.709, 113.121,  73.763,  83.258, 269.473, -240.869])),
        ('pick_upT6',   _to_rad([ 75.129, 143.812,  20.841, 105.529, 269.485, -236.448])),
        ('pickupT7',    _to_rad([ 78.421, 139.311,  49.086,  81.779, 269.483, -237.186])),
        ('pickupT8',    _to_rad([ 78.758, 141.894,  58.083,  70.202, 269.484, -236.849])),
        ('grip',        'OPEN'),
        ('uplast',      _to_rad([ 86.377, 142.558,  18.014, 109.671, 269.517, -229.230])),
        ('end_home',    _to_rad([  0.000,   0.344, 146.804, 114.819, 100.246,   0.000])),
    ]


# =============================================================================
# PART 2 — all_40_joint_angles.xlsx
# =============================================================================
def _build_part2():
    return [
        ('starting',         _to_rad([  0.000,  90.000,   0.000,  90.000, 180.000,   0.000])),
        ('pickTloop',        _to_rad([  0.000,  90.000,  90.000,  90.000, 270.000,  29.673])),
        ('reachingbox',      _to_rad([ 59.007, 130.440,  33.368, 106.192, 270.000,  18.880])),
        ('reaching_heightT', _to_rad([ 72.388, 112.608,  62.246,  95.146, 270.000, 120.383])),
        ('reachingt1',       _to_rad([ 75.714, 144.037,   7.163, 118.800, 270.000, 123.708])),
        ('reachedT',         _to_rad([ 78.418, 134.905,  60.795,  74.300, 270.000, 126.413])),
        ('pickinT',          _to_rad([ 78.212, 139.701,  64.379,  65.920, 270.000, 126.207])),
        ('grip',             'CLOSE'),
        ('upthet',           _to_rad([ 78.212, 134.064,  53.442,  82.494, 270.000, 126.207])),
        ('movethet',         _to_rad([ 72.483,  97.608,  87.339,  85.053, 270.000, 120.478])),
        ('movet2',           _to_rad([ 44.076, 146.466,   8.325, 115.209, 270.000,  92.070])),
        ('movet3',           _to_rad([  4.193, 113.182,  66.948,  89.870, 270.000, 142.444])),
        ('movet4',           _to_rad([  6.607, 134.458,  31.798, 103.744, 270.000, 144.858])),
        ('movet5',           _to_rad([  3.898, 134.012,  72.561,  63.428, 270.000, 142.149])),
        ('grip',             'OPEN'),
        ('pickingi',         _to_rad([  3.898, 130.390,  31.235, 108.374, 270.000, 142.149])),
        ('picki2',           _to_rad([ 73.496,  75.927,  99.941,  94.132, 270.000, 121.603])),
        ('picki3',           _to_rad([ 83.745, 136.995,  18.918, 114.087, 270.000, 131.852])),
        ('pickireaching',    _to_rad([ 86.954, 133.190,  66.604,  70.206, 270.000, 135.061])),
        ('pickireached',     _to_rad([ 87.020, 136.659,  70.417,  62.924, 270.000, 135.127])),
        ('grip',             'CLOSE'),
        ('lifti',            _to_rad([ 87.020, 134.053,  67.867,  68.081, 270.000, 135.127])),
        ('movingi',          _to_rad([ 87.020, 133.602,  25.518, 110.880, 270.000, 135.127])),
        ('movei1',           _to_rad([  6.356, 107.941,  75.159,  86.901, 270.000, 147.192])),
        ('reachingiplace',   _to_rad([  5.152, 131.216,  38.508, 100.276, 270.000, 145.989])),
        ('placingpostioni',  _to_rad([  3.798, 131.609,  66.888,  71.502, 270.000, 140.076])),
        ('grip',             'OPEN'),
        ('moveupfori',       _to_rad([  3.798, 128.337,  53.610,  88.054, 270.000, 140.076])),
        ('movebackfori',     _to_rad([ 31.714, 142.134,  28.361,  99.504, 270.000, 167.992])),
        ('movefori1',        _to_rad([101.321,  74.122, 119.831,  76.048, 270.000, 150.495])),
        ('movefori2',        _to_rad([ 97.082, 103.939,  90.614,  75.447, 270.000, 146.256])),
        ('oni',              _to_rad([ 94.005, 128.238,  53.780,  87.982, 270.000, 143.179])),
        ('reachingi',        _to_rad([ 96.114, 132.604,  69.439,  67.957, 270.000, 145.288])),
        ('picki_A',          _to_rad([ 96.118, 134.032,  73.605,  62.363, 270.000, 141.702])),
        ('picki2_A',         _to_rad([ 96.118, 135.594,  74.714,  59.692, 270.000, 141.702])),
        ('grip',             'CLOSE'),
        ('pickifrom',        _to_rad([ 96.118, 132.699,  72.390,  64.911, 270.000, 141.702])),
        ('pickifrom2',       _to_rad([ 96.118, 129.400,  33.605, 106.996, 270.000, 141.702])),
        ('movingifrom',      _to_rad([ 98.948,  92.626,  85.755,  91.619, 270.000, 144.531])),
        ('movingfrom1',      _to_rad([ 51.903, 130.004,  32.528, 107.468, 270.000,   9.236])),
        ('reachingito',      _to_rad([ 12.521,  97.135,  80.797,  92.067, 270.000, -30.146])),
        ('reachingheight1',  _to_rad([  5.556, 128.255,  35.624, 106.122, 270.000, -37.111])),
        ('reachingiheight2', _to_rad([  3.761, 128.617,  65.015,  75.877, 269.968, -41.375])),
        ('reaching_precise', _to_rad([  3.750, 128.909,  64.357,  76.734, 270.000, -41.386])),
        ('grip',             'OPEN'),
        ('up',               _to_rad([  3.750, 128.120,  42.242,  99.638, 270.000, -41.386])),
    ]


# =============================================================================
# Helpers
# =============================================================================

def _all_labels(sequence):
    """Return list of all arm-move labels (no 'grip' entries)."""
    return [lbl for lbl, val in sequence if lbl != 'grip']


def _execute_sequence(node: IITLogoPickAndPlace, sequence: list, start_idx: int = 0):
    """
    Execute a sequence from start_idx.
    sequence entries: ('label', positions_rad) or ('grip', 'OPEN'/'CLOSE')
    Returns False if stop_flag was set.
    """
    arm_idx = 0  # counts only arm-move entries
    for lbl, val in sequence:
        if node.stop_flag:
            return False
        if lbl == 'grip':
            if val == 'OPEN':
                node.move_gripper(GRIPPER_OPEN)
            else:
                node.move_gripper(GRIPPER_CLOSE)
        else:
            if arm_idx >= start_idx:
                node.move_to(val, label=lbl)
            arm_idx += 1
    return not node.stop_flag


# =============================================================================
# Main routine
# =============================================================================

def execute_logo_routine(node: IITLogoPickAndPlace):
    if not node.wait_for_servers():
        return

    part1 = _build_part1()
    part2 = _build_part2()

    # ── Startup prompt ────────────────────────────────────────────────────────
    print('\n' + '='*60)
    print('  IIT Logo Pick-and-Place  (Ctrl+C to stop after any move)')
    print('='*60)
    print('CONVEYER mode starts the routine from waypoint #1.')
    answer = input('  Conveyer? (y/n): ').strip().lower()

    start_idx_p1 = 0   # default: from beginning of Part 1
    start_idx_p2 = 0
    first_cycle_part = 1  # which part to start on

    if answer != 'y':
        # n → skip Part 1, jump straight to Part 2 from the beginning
        first_cycle_part = 2
        start_idx_p2    = 0
        print('  → Starting directly from Part 2 (all_40_joint_angles).')

    print('\n  Press Ctrl+C at any time to stop after the current waypoint.\n')
    print('='*60 + '\n')

    # ── Infinite loop ─────────────────────────────────────────────────────────
    cycle = 1
    is_first = True

    while not node.stop_flag and rclpy.ok():
        node.get_logger().info(f'=== CYCLE {cycle} START ===')

        if is_first and first_cycle_part == 2:
            # Skip Part 1 entirely on first cycle, jump into Part 2
            node.get_logger().info(f'--- Cycle {cycle}: Skipping to Part 2 ---')
            ok = _execute_sequence(node, part2, start_idx=start_idx_p2)
        else:
            # Part 1
            node.get_logger().info(f'--- Cycle {cycle}: Part 1 ---')
            s1 = start_idx_p1 if is_first else 0
            ok = _execute_sequence(node, part1, start_idx=s1)

            if ok and not node.stop_flag:
                # Part 2
                node.get_logger().info(f'--- Cycle {cycle}: Part 2 ---')
                s2 = start_idx_p2 if (is_first and first_cycle_part == 1) else 0
                # Note: if first_cycle_part==1, start_idx_p2 is always 0 anyway
                ok = _execute_sequence(node, part2, start_idx=0)

        is_first = False
        # After first cycle always start both parts from the beginning
        start_idx_p1 = 0
        start_idx_p2 = 0

        if not ok or node.stop_flag:
            break

        node.get_logger().info(f'=== CYCLE {cycle} COMPLETE — repeating… ===')
        cycle += 1

    node.get_logger().info('=== IIT Logo routine STOPPED ===')


# =============================================================================
# Entry point
# =============================================================================

def main(args=None):
    rclpy.init(args=args)
    node = IITLogoPickAndPlace()
    try:
        execute_logo_routine(node)
    except KeyboardInterrupt:
        node.get_logger().info('Ctrl+C received — stopping after current move…')
        node.stop_flag = True
        time.sleep(1.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

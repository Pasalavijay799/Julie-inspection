# JAKA Zu5 Visual Inspection: Node Architecture & Communication

This document outlines the specific ROS 2 nodes running in the JAKA visual inspection workspace. It details their core functionalities, the topics/services they expose, and specifically how they handle real-world physical connections to the robot hardware.

---

## 1. `jaka_driver` (Hardware Interface Node)
**Package**: `jaka_driver` | **Language**: C++  

This node serves as the primary gateway to the physical JAKA Zu5 robotic arm.

### Real Robot Connection Logic
The JAKA Zu5 controller does not run ROS directly. Instead, this node uses the proprietary JAKA C++ SDK (`JAKAZuRobot.h`).
- **Initialization**: It establishes a standard TCP/IP socket connection to the robot controller via `robot.login_in(robot_ip, false)`. 
- **Start-up Sequence**: It systematically turns on power via `robot.power_on()` and enables the servos via `robot.enable_robot()`.
- **Servoing Control**: To execute continuous fluid motion, the node must activate a specific SDK mode using `robot.servo_move_enable(true)`.

### Exposed Interfaces
- **Publishers**:
  - `/joint_states` (`sensor_msgs/JointState`): Queries `robot.get_joint_position()` and publishes the 6 joint angles in radians.
  - `/tool_position` (`geometry_msgs/TwistStamped`): Queries `robot.get_tcp_position()` and publishes Cartesian endpoint data.
  - `/robot_states` (`jaka_msgs/RobotMsg`): Publishes diagnostic modes (in collision, in position, drag mode, emergency stop).
- **Services**:
  - `linear_move`, `joint_move`, `jog`: Services to jog or move the robot blockingly using the SDK.
  - `servo_p`, `servo_j`: Step-wise commands for fluid motion controllers.
  - `set_payload`, `set_toolFrame`: Dynamic variable updates to the controller.

---

## 2. `onrobot_2fg7_driver` (Gripper Action Server)
**Package**: `jaka_driver` | **Language**: Python

This node controls the OnRobot 2FG7 end effector attached to the robot.

### Real Robot Connection Logic
The gripper is wired to an OnRobot Compute Box, which exposes an HTTP REST API over the local network, bypassing traditional ROS hardware/modbus interfaces.
- **Connection**: It requires only the Compute Box IP (`gripper_ip`) and an HTTP port (`80`).
- **Command Dispatch**: When a close command is given, it transforms the target meter distance into a millimeter integer, formats a string URI (`GET /api/dc/twofg/grip_external/...`), and executes a standard network library `urllib.request.urlopen()`. If the compute box returns `0`, the command was valid.

### Exposed Interfaces
- **Action Server**: 
  - `/gripper_controller/follow_joint_trajectory` (`control_msgs/action/FollowJointTrajectory`): Listens for action goals containing `left_finger_joint` positions.
- **Publishers**:
  - `/joint_states` (`sensor_msgs/JointState`): Publishes the mimicked prismatic joint positions of the gripper fingers.
- **Services**:
  - `~/set_gripper` (`std_srvs/SetBool`): A simple boolean switch to fully open (True) or fully close (False) the gripper without needing a full trajectory goal.

---

## 3. `moveit_server` (Trajectory Execution Node)
**Package**: `jaka_planner` | **Language**: C++

This node acts as the bridge connecting the standard MoveIt 2 motion planner stack with the JAKA SDK.

### Real Robot Connection Logic
Rather than calling the `jaka_driver` ROS services, this node independently accesses the SDK (`JAKAZuRobot.h`) directly to reduce latency during real-time tracking.
- **Connection**: Replicates the `login_in`, `power_on`, and `enable_robot` sequences explicitly.
- **Pipeline Execution**: 
  - Iterates over MoveIt's trajectory response `points`.
  - Determines the time interval (`time_from_start`) of the current target point versus the previous one (`dt`).
  - The JAKA controller updates on a strict 8ms loop. The node calculates how many 8ms SDK-ticks are required (`step_num = dt / 0.008f`).
  - Calls `robot.servo_j(&joint_pose, MoveMode::ABS, step_num)` to command the physical hardware.

### Exposed Interfaces
- **Action Server**:
  - `/jaka_zu3_controller/follow_joint_trajectory` (Note: Default namespace is "zu3" unless overridden).
- **Publishers**:
  - `/joint_states` (`sensor_msgs/JointState`): Duplicates joint publishing directly from its own SDK polling loop.

---

## 4. `auto_planner_node` (Vision Orchestrator)
**Package**: `jaka_visual_inspection` | **Language**: Python

This is the high-level brain. It does *not* possess direct connection logic to robot hardware. Instead, it coordinates perception data to create action goals for MoveIt.

### Real Robot Connection Logic
- **Intel RealSense**: Interfaces with physical camera hardware indirectly via the `pyrealsense2` or standard `realsense2_camera` ROS wrappers to assemble point clouds.
- **Kinematics**: Uses `moveit_interface.py` to command MoveGroup actions via the ROS 2 standard `MoveItCpp` or `moveit_commander` wrappers.

### Exposed Interfaces
- **Broadcasters**:
  - `tf2_ros.TransformBroadcaster` and `tf2_ros.StaticTransformBroadcaster`: When new visual inspection profiles are extracted, this node publishes `camera_target_0`, `camera_target_1`, etc., as actual static transforms over the `/tf_static` topic so that MoveIt can natively resolve target IK solutions relative to the base link.

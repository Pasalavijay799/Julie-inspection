# JAKA Zu5: OnRobot 2FG7 Gripper Integration Guide

This document explains, step-by-step, every file and logic block involved in integrating, connecting, and controlling the OnRobot 2FG7 Gripper within the JAKA Zu5 workspace.

---

## 1. Physical Connection & Networking

The OnRobot 2FG7 does not communicate natively through the JAKA robotic arm controller or via standard ROS serial/modbus to your PC. Instead:
- **Compute Box**: The gripper is hardwired into an intermediary **OnRobot Compute Box**.
- **Network**: The Compute Box is connected via Ethernet (TCP/IP) to the host PC (running ROS 2).
- **Communication Protocol**: The Compute Box exposes an HTTP REST API on Port `80`. The driver controls the gripper by sending specially formatted URL requests.

---

## 2. Model & URDF Integration (`jaka_description`)

To ensure MoveIt and RViz know the gripper exists and where it is attached, the URDF file must define its physical properties.

### File: `jaka_description/urdf/jaka_zu5.urdf`
- **Mounting (Fixed Joint)**: 
  The base of the gripper is attached to the flange of the JAKA arm (`Link_6`).
  ```xml
  <joint name="gripper_base_joint" type="fixed">
    <origin xyz="0 0 0" rpy="0 0 -2.26893"/> <!-- Yaw offset to match reality -->
    <parent link="Link_6"/>
    <child link="gripper_base_link"/>
  </joint>
  ```
- **Fingers (Prismatic Joints)**: 
  The 2FG7 is a parallel gripper, so it has two fingers that slide linearly (`prismatic`).
  - `left_finger_joint`: The actual actuated joint. It has limits `[0.0, 0.019]` (meters).
  - `right_finger_joint`: A `<mimic>` joint. It copies the state of the left finger but slides in the opposite direction.

---

## 3. ROS 2 Control Hardware Abstraction (`jaka_zu5_moveit_config`)

MoveIt needs to know how to send and receive commands to those gripper joints.

### File: `jaka_zu5_moveit_config/config/jaka_zu5.ros2_control.xacro`
- **Command Interfaces**: MoveIt declares that it intends to send `position` commands to the `left_finger_joint` and `right_finger_joint`.
- **State Interfaces**: The driver will report back the `position` and `velocity` of those joints.
```xml
<joint name="left_finger_joint">
    <command_interface name="position"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
</joint>
<!-- The right finger mimics the left finger with a multiplier of 1.0 -->
<joint name="right_finger_joint">
    <param name="mimic">left_finger_joint</param>
...
<!-- This binds the logical URDF joints to the MoveIt Controller Manager -->
```

---

## 4. The Action Server Driver (`jaka_driver`)

MoveIt plans a sequence of "open" or "close" positions over time and sends them as a `FollowJointTrajectory` ROS Action. The driver must intercept this action and convert it into real-world electrical commands.

### File: `jaka_driver/scripts/onrobot_2fg7_driver.py`
This is a standalone Python ROS 2 node that handles all gripper communication.

#### A. Node Parameters configuration
The node expects parameters pointing to the Compute Box:
- `gripper_ip`: e.g., `"192.168.1.100"`
- `http_port`: `80`
- `device_id`: `0`
- `grip_mode`: `"external"` (or `"internal"`)
- `default_force_n`: Force in Newtons (e.g., `40N`)

#### B. The Action Execution Loop
When MoveIt triggers a grasp, the Action Server (`execute_callback`) activates:
1. **Extract final target**: The compute box doesn't need a smooth spline; it just needs the final target position. The driver looks at `trajectory.points[-1]`.
2. **Meter to Millimeter Conversion**: 
   MoveIt uses SI units (`0.019` meters). The REST API requires millimeters (`37` mm). 
   The function `_position_m_to_width_mm` calculates this ratio linearly.
3. **HTTP REST Command (`_command_gripper_position`)**:
   It constructs a URL targeting the `grip_external` endpoint.
   `GET http://192.168.1.100:80/api/dc/twofg/grip_external/0/37/40/50`
   *(This tells the Compute Box: Device 0, Width 37mm, Force 40N, Speed 50%)*
4. **Validation**: It uses python's `urllib.request.urlopen` to send the HTTP GET request. The box returns `"0"` if the command was successfully acknowledged.

#### C. Telemetry Loop (`_publish_joint_state`)
To keep MoveIt and RViz updated so the virtual 3D model fingers visually match the real physical fingers, a timer fires every `0.1s` (10Hz). It creates a `sensor_msgs/JointState` message containing the `_current_position_m` and publishes it to the standard `/joint_states` topic alongside the JAKA arm joints.

#### D. The Setting Service (`~/set_gripper`)
For rapid testing, the node exposes a custom service taking a `SetBool`.
- `True = Open`
- `False = Close`
This bypasses MoveIt entirely and directly fires the HTTP GET request to the Compute Box for instant debugging.

# JAKA Zu5 Visual Inspection: Core Logic and Features Documentation

This document provides an in-depth explanation of the core features and underlying logic of the `jaka_ws` workspace. It acts as a reference guide for the essential pipelines powering the autonomous inspection workflow.

---

## 1. Hardware Communication Logic

The workspace bridges the gap between ROS 2 / MoveIt standard commands and the proprietary hardware interfaces for the robot arm and the gripper.

### JAKA Zu5 Robot Arm Control (`jaka_driver`)
The JAKA robot is controlled via a closed-source C++ SDK (`JAKAZuRobot.h`). The driver node (`jaka_driver.cpp`) acts as a persistent ROS 2 daemon that queries the real-time status of the arm and exposes custom services for direct control. 
- **Initialization**: Login to the robot controller via TCP/IP. Turns on power and enables the servos.
- **Servo Mode Control**: High-frequency continuous joint commands are sent using `robot.servo_j(...)`.
- **Telemetry**: Continuously publishes `/joint_states` and `ToolPosition` (`geometry_msgs/TwistStamped`) for tracking. 

### OnRobot 2FG7 Gripper Control (`onrobot_2fg7_driver.py`)
Instead of Modbus, this node uses the OnRobot Compute Box HTTP REST API. 
- **Action Server**: Implements a standard `FollowJointTrajectory` action server, making it fully compatible with standard ROS 2 grip pipelines.
- **Conversion Logic**: MoveIt plans in terms of prismatic joint translations (meters) but the REST API expects millimeters (width). The driver handles this conversion linearly:
  - Width (mm) is mapped between `0.0m` (open) and `0.019m` (closed).
- **Execution Request**: It issues an HTTP GET request such as:
  `GET /api/dc/twofg/grip_external/{id}/{width_mm}/{force_N}/{speed_pct}`

---

## 2. Motion Planning and Execution Logic

Standard MoveIt 2 pipelines are utilized for generating collision-free inverse kinematics, which are then fed into the JAKA driver.

### MoveIt Action Server (`jaka_planner/moveit_server.cpp`)
When MoveIt 2 computes a joint trajectory path, it sends a `FollowJointTrajectory` goal. 
- **Trajectory Interpolation**: MoveIt provides waypoints with timestamps (`time_from_start`). The server calculates the time elapsed between points (`dt`).
- **Timing Constraint Matching**: The JAKA Controller SDK requires a step count. Assuming an internal control loop of `8ms` (0.008s), the server calculates the number of steps to spend on each motion segment: `step_num = dt / 0.008`.
- **Servoing**: It iterates through the waypoints and commands `servo_j` point-by-point, waiting until the joint tolerances (`±0.2` degrees) are satisfied.

---

## 3. Visual Inspection Pipeline

The core logic of the visual inspection application is wrapped in `jaka_visual_inspection/auto_planner_node.py`. It runs a state machine to conduct either manual scans or automated replay routines.

### Point Cloud Capture and filtering (`point_cloud_processor.py`)
1. **Gathering Data**: Uses the `RealSense` wrapper to export depth frames as `open3d.geometry.PointCloud`.
2. **Cropping & Trimming**: Bounds the point cloud logically (removes boundaries out of the work envelope or tool field-of-view). 
3. **Base Plane Removal**: Uses RANSAC plane segmentation to identify and eliminate the dominant planar surface (the table/floor), isolating only the raised objects sitting on it.

### DBSCAN Clustering (`clustering.py`)
Once the table is removed, remaining points are distinct objects.
1. Uses Density-Based Spatial Clustering of Applications with Noise (DBSCAN) with a parameterized `eps` (radius) and `min_points`.
2. Groups adjacent points into individual cluster IDs. Small noise clusters are discarded based on a `discard_threshold`.
3. In Manual Mode, the remaining clusters are captured as image previews and displayed via a GUI prompt for the user to manually select the object to inspect.

### Profile Normalization (`profile_generator.py`)
To inspect the object, the system calculates grid profiles.
- It finds the maximum bounds of the selected cluster cloud along the X and Y axes.
- It partitions the object surface into specific evenly spaced discrete profiles (e.g. slicing through the middle of the object X-axis and Y-axis).

---

## 4. Geometric Targets and Transforms (`target_generator.py` & `tf_utils.py`)

Generating coordinate poses for MoveIt to trace requires matching the exact surface contour of the point cloud while maintaining the camera normal.

1. **Surface Normals Extraction**: For each profile grid line along the point cloud, Open3D calculates the local surface normal vectors.
2. **Camera Alignment Logic**:
   - `get_rotation_from_normal`: Takes the extracted 3D normal vector from the surface. It uses cross products and dot products against the base frame unit vectors to generate a complete Scipy `Rotation` matrix that perfectly aligns the `-Z` axis with the inverse of the surface normal (causing the camera lens to look directly flat at the contoured surface).
3. **Z-Offset Shifting**: The camera must be held at a fixed standoff distance (e.g. 10cm). The code offsets the target coordinate along the calculated Z normal.
4. **TF2 Publishing**: Valid target coordinates are filtered for redundancy, converted from 4x4 homogenous matrices back to quaternions, and broadcasted to TF2 as static frames (`camera_target_0`, `camera_target_1`, etc.) so the user can preview the inspection path in RViz before confirming execution.

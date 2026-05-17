# JAKA Zu5 Visual Inspection: Beginner's Getting Started Guide

Welcome to the JAKA Visual Inspection workspace! This guide is designed to bridge the gap between understanding the code (covered in the other documentation files) and actually running the robot on day one.

If you are new to ROS 2 or this specific robotic cell, follow these steps to safely power up, launch, and troubleshoot the system.

---

## 1. Physical Prerequisites & Safety Checks
Before running any code, ensure the physical cell is ready:

1. **E-Stop Check**: Ensure the Emergency Stop button on the JAKA teach pendant and the robot control box is completely released (twisted outward).
2. **Network Connection**: 
   - Ensure the Ethernet cable from your PC is plugged into the network switch.
   - The JAKA Controller and the OnRobot Compute Box must also be connected to this switch.
3. **Ping Test**: Open a terminal on your Ubuntu PC and verify communication:
   ```bash
   ping 192.168.1.50   # Replace with your JAKA robot IP
   ping 192.168.1.100  # Replace with your OnRobot Compute box IP
   ```
   *If the pings fail, check your PC's ipv4 subnet settings (e.g., ensure your PC IP is `192.168.1.xxx`).*

---

## 2. Building the Workspace
Whenever you make changes to the C++ or Python code, you must recompile the workspace.

1. Open a terminal and navigate to your workspace (usually `~/jaka_ws` or `~/Pictures/jaka_ws`).
2. Source the underlying ROS 2 installation (e.g., Humble or Foxy):
   ```bash
   source /opt/ros/humble/setup.bash
   ```
3. Build the packages using `colcon`:
   ```bash
   colcon build --symlink-install
   ```
   *(The `--symlink-install` flag is highly recommended for Python files, as it allows you to edit the Python code without having to rebuild every time!)*
4. Source your newly built workspace:
   ```bash
   source install/setup.bash
   ```

---

## 3. The Launch Sequence
ROS 2 systems are typically brought up in a specific order. Open a new terminal for each step (remembering to run `source install/setup.bash` in every new terminal).

### Step 1: Launch the Robot Driver & MoveIt 2
This step connects to the real robot arm, loads the URDF models, and starts the MoveIt trajectory planner.
```bash
ros2 launch jaka_planner real_robot.launch.py robot_ip:=192.168.1.50
```
*(Note: If the JAKA arm clicks and powers on, the connection was successful. A virtual RViz window should appear showing the robot).*

### Step 2: Launch the Gripper Driver
In a second terminal, start the OnRobot HTTP driver so the system can open and close the fingers.
```bash
ros2 run jaka_driver onrobot_2fg7_driver --ros-args -p gripper_ip:=192.168.1.100
```
*(You should see a log message saying "OnRobot 2FG7 REST driver ready".)*

### Step 3: Run the Visual Inspection Application
In a third terminal, launch the main graphical brain of the operation. This will open the PySimpleGUI menus.
```bash
ros2 run jaka_visual_inspection auto_planner_node
```

---

## 4. First Run Walkthrough (Manual Mode)

Once the GUI opens from Step 3, try scanning an object:
1. Click **Manual Mode** on the front menu.
2. The initial pose prompt shows images of the robot. Click the first image. The robot will autonomously move to a hover position over the table. *(Keep a hand near the E-Stop!)*
3. A 3D Open3D window will pop up showing what the Intel RealSense camera sees. The table floor will be automatically removed (RANSAC). Close this window to proceed.
4. Thumbnail images of clustered objects (isolated by DBSCAN) will appear. Click the object you want to inspect.
5. In the Profile Selection window, check the boxes for the exact grid lines you want the camera to trace along the object's surface. Click **Proceed**.
6. Switch your view to the **RViz window** opened in Step 1. You will see colored target axes appear over your object. These are the generated TF coordinates `camera_target_0`, `camera_target_1`, etc.
7. The robot will now physically execute the sweep, keeping the camera lens perfectly flat against the object's contours!

---

## 5. Common Errors and Troubleshooting

### 1. "Could not connect to gripper" / Compute Box Errors
- **Cause**: The HTTP REST API ping timed out or returned a non-zero code.
- **Fix**: Verify the `gripper_ip` parameter. Open a web browser on your PC and type `http://192.168.1.100`. If the OnRobot Web UI doesn't load, your network configuration is wrong, or the Ethernet cable is unplugged.

### 2. "Action client not connected to action server" (MoveIt Error)
- **Cause**: MoveIt attempted to send a trajectory, but the `jaka_driver` or `moveit_server` node crashed or hasn't fully loaded yet.
- **Fix**: Check Terminal 1. Ensure `jaka_driver` successfully printed "Login Success". Restart the launch file if necessary.

### 3. Open3D Point Cloud is completely empty
- **Cause**: The RealSense camera failed to export data, or the physical work envelope limits cropped out everything in the scene.
- **Fix**: Open the **Settings** menu on the Front GUI and check the `trim_base`, `offset_y`, and `offset_z` bounds. Ensure the object isn't physically outside this bounding box.

### 4. "ERR_EMERGENCY_PRESSED" from JAKA SDK
- **Cause**: The physical E-Stop is pushed down.
- **Fix**: Twist to release the physical red E-stop button. You may need to restart the ROS 2 driver terminal so it can re-attempt `robot.power_on()` safely.

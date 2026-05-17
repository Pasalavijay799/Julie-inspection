# JAKA Visual Inspection Application Architecture

This document explains the overarching logic, workflow, and Graphical User Interface (GUI) system that powers the `jaka_visual_inspection` package.

---

## 1. What is `jaka_visual_inspection`?

The `jaka_visual_inspection` package is the autonomous "brain" of the robotic cell. Rather than just being a simple ROS 2 node that loops indefinitely, it behaves like an interactive desktop application. 

It is entirely driven by a central orchestrator script, `auto_planner_node.py`, which integrates Computer Vision (Open3D + Intel RealSense), GUI menus (PySimpleGUI), and robotic motion (MoveIt 2).

### The Subprocess GUI Architecture
ROS 2 nodes running in the background typically struggle with rendering traditional blocking GUI windows (like Tkinter or Qt) because the ROS executors need to constantly spin and process callbacks.

To solve this, the JAKA Visual Inspection app splits the GUI from the ROS node:
1. `auto_planner_node.py` handles all the actual ROS 2 logging, TF broadcasting, and MoveIt commands.
2. When the node needs user input (e.g., choosing which object to scan), it spawns a separate, temporary child process using python's `subprocess` library (e.g., `subprocess.run(['python3', 'Cluster_selection_gui.py'])`).
3. The graphical PySimpleGUI window opens, the user interacts with it, and the script prints the chosen configuration to `stdout`.
4. The ROS 2 node captures that `stdout`, parses it using `ast.literal_eval`, and resumes the robotic pipeline.

---

## 2. The Interactive GUI Workflow

When you start the `auto_planner_node`, it immediately drops you into a multi-mode workflow.

### Mode 1: Front Menu (`Front_gui.py`)
This is the starting landing page. It provides four large interactive choices:
- **Auto Mode**: For automated repetitive scanning.
- **Manual Mode**: A step-by-step guided mode for inspecting novel objects.
- **Replay Mode**: Quickly execute a previously tested and saved inspection path.
- **Settings**: A global configuration editor.

### Mode 2: Settings (`Settings_gui.py`)
This menu edits the `config.ini` file. It allows the user to dynamically adjust:
- **DBSCAN Parameters**: `eps` (radius) and `min_points` for clustering logic.
- **Vision Spacing**: How far apart the inspection targets should be placed on the object.
- **Z-Offset**: The standoff distance between the camera lens and the object surface during the inspection scan.

### Mode 3: Manual Inspection Workflow (`Manual_gui.py`)
The Manual Mode is the core feature of the application. It walks the operator through a detailed, multi-step sequence to construct an inspection path from scratch.

1. **Initial Pose Selection**: A GUI opens showing reference images of the robotic cell. The user clicks an image to command the robot into an initial "bird's-eye view" scanning position over the table.
2. **First Scan & RANSAC**: The RealSense camera captures the scene. A 3D window (Open3D) pops up showing the filtered point cloud with the table removed using RANSAC plane segmentation.
3. **Cluster Selection (`Cluster_selection_gui.py`)**: The DBSCAN clustering algorithm separates distinct objects on the table. The GUI presents 2D thumbnail screenshots of each clustered object. The user clicks on the exact object they wish to physically inspect.
4. **Profile Selection (`Profile_selection_gui.py`)**: The chosen object is analyzed. The application draws horizontal and vertical grid "profiles" across the object. The user is presented with a GUI containing checkboxes to select exactly which slices of the object they want the camera to trace.
5. **Path Execution**: The orchestrator computes the Inverse Kinematics for those exact profiles, ensures the camera remains perfectly tangential to the surface contour, and commands MoveIt 2 to execute the sweep. The user can opt to save this path for later use.

### Mode 4: Auto Mode (`Auto_gui.py`)
In Auto mode, the user configures the settings *once* upfront. They select the initial robotic pose, whether to scan the `X` contours, `Y` contours, or `Both`, and how many loop iterations to run.

Once started, the robot will recursively snap a picture, auto-select the largest cluster (bypassing the cluster selection GUI), auto-generate the central profiles (bypassing the profile GUI), and execute the inspection scan repetitively without needing user confirmation.

### Mode 5: Replay Mode (`Replay_gui.py`)
If a user created a perfect inspection path in Manual Mode and saved it to `~/jaka_targets/`, they can use Replay Mode.
A GUI will open listing the saved `.npy` trajectory files. The user can load a file, optionally apply global X/Y/Z translation offsets (if the workpiece on the table moved slightly), and immediately watch the robot execute the scan without turning on the camera depth sensors.

---

## 3. How the Physical Inspection Works (The Math)

The ultimate goal of the inspection GUI pipeline is to generate a list of Cartesian coordinates (`x, y, z, rx, ry, rz`) for MoveIt to plan through.

1. **Tangent Aligning**: When profiling an object, the system relies strictly on 3D surface normals. An algorithm parses the selected profile line and calculates the normal vector of every point along the object's curvature.
2. **Rotation Matrix Generation**: Using cross-products and `scipy.spatial.transform.Rotation`, it forces the `-Z` axis of the robot's tool flange (the camera lens) to directly oppose the normal vector. This guarantees the camera lens stays fundamentally parallel to the curved surface of the object during the entire sweep.
3. **TF Publication**: To visualize the path before executing, the system takes the list of coordinates and broadcasts them as physical static frames (e.g., `camera_target_0`, `camera_target_1`) to ROS 2's `/tf` topic viewable in RViz.

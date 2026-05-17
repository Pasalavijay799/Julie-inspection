"""
Auto Path Planner Node for JAKA ZU5.

Main ROS2 node that orchestrates the visual inspection pipeline:
1. GUI mode selection (Manual/Auto/Replay/Settings)
2. Initial pose selection and movement
3. Point cloud capture and processing
4. DBSCAN clustering and object selection
5. Profile extraction
6. Target coordinate generation
7. MoveIt2 planning and execution
8. Target saving
"""

import rclpy
from rclpy.node import Node
import tf2_ros
import numpy as np
import open3d as o3d
import os
import sys
import subprocess
import time
from ast import literal_eval
from ament_index_python.packages import get_package_share_directory

from .point_cloud_processor import PointCloudProcessor
from .clustering import (
    cluster_point_cloud, trim_cluster, fetch_cloud_image, combine_clusters
)
from .profile_generator import (
    get_profile_counts, extract_profiles, get_xy_angles_from_pc
)
from .target_generator import (
    generate_coordinates, generate_final_coordinates, filter_coordinates
)
from .moveit_interface import MoveItInterface
from .config_manager import ConfigManager
from . import tf_utils


class AutoPlannerNode(Node):
    """Main auto path planner node for JAKA ZU5 visual inspection."""

    def __init__(self):
        super().__init__('auto_planner_node')

        # TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf_static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        # Components
        self.pc_processor = PointCloudProcessor(self)
        self.moveit = MoveItInterface(self)

        # Config
        pkg_share = get_package_share_directory('jaka_visual_inspection')
        config_path = os.path.join(pkg_share, 'config', 'config.ini')
        self.config = ConfigManager(config_path)
        self.gui_dir = os.path.join(pkg_share, 'gui')

        self.get_logger().info("=== Auto Path Planner for JAKA ZU5 initialized ===")

    def run(self):
        """Main execution loop."""
        while rclpy.ok():
            # Show front GUI
            mode = self._run_front_gui()

            if mode == -1:
                self.get_logger().info("Exiting...")
                break
            elif mode == 0:
                self._run_replay_mode()
            elif mode == 1:
                self._run_manual_mode()
            elif mode == 2:
                self._run_auto_mode()
            elif mode == 3:
                self._run_settings()

    # ======================== Mode Handlers ========================

    def _run_manual_mode(self):
        """Execute manual scanning mode."""
        s = self.config.settings

        # Select initial pose
        pose_idx, manual_offset, tgt_save, exit_flag = self._run_manual_gui(
            s['manual_offset'], s['tgt_save']
        )
        if exit_flag:
            return

        s['manual_offset'] = manual_offset
        s['tgt_save'] = tgt_save

        # Move to selected initial pose
        self._move_to_initial_pose(pose_idx)

        # Wait for robot to settle
        time.sleep(1.0)

        # Capture and process point cloud
        self.get_logger().info("Capturing point cloud...")
        downpcd = self.pc_processor.load_point_cloud(
            samples=s['samples'],
            offset_y=s['offset_y'],
            offset_z=s['offset_z'],
            manual_offset=s['manual_offset'],
            spacing=s['spacing'],
            trim_base=s['trim_base'],
            tf_buffer=self.tf_buffer,
            dbug=s['dbug']
        )

        if len(downpcd.points) == 0:
            self.get_logger().error("No points in captured point cloud!")
            return

        # === Open3D Preview: Show filtered + downsampled point cloud ===
        if s['dbug']:
            mesh = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
            self.get_logger().info("Showing filtered point cloud (close window to continue)...")
            o3d.visualization.draw_geometries(
                [downpcd, mesh],
                window_name='07_Filtered Pointcloud: Estimate surface normals',
                point_show_normal=True
            )

        # Cluster
        clouds = cluster_point_cloud(
            downpcd, eps=s['eps'], min_points=s['min_points'],
            discard_threshold=s['cluster_discard'],
            logger=self.get_logger()
        )

        # Select clusters via GUI
        selected_indices = self._run_cluster_gui(clouds)
        selected_pc = combine_clusters(clouds, selected_indices)

        # Trim cluster edges
        if s['cluster_trim'] > 0:
            selected_pc = trim_cluster(selected_pc, s['cluster_trim'])

        # Ensure normals are estimated
        if len(selected_pc.normals) == 0:
            selected_pc.estimate_normals(
                search_param=o3d.geometry.KDTreeSearchParamHybrid(
                    radius=0.02, max_nn=30
                )
            )
            selected_pc.orient_normals_towards_camera_location(np.array([0., 0., 0.]))

        # === Open3D Preview: Show selected cluster with normals ===
        if s['dbug']:
            mesh = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
            self.get_logger().info("Showing selected cluster (close window to continue)...")
            o3d.visualization.draw_geometries(
                [selected_pc, mesh],
                window_name='08a_Selected Cluster',
                point_show_normal=True
            )

        # Get profile counts
        x_count, y_count, x_bounds, y_bounds = get_profile_counts(
            selected_pc, s['spacing']
        )
        self.get_logger().info(f"Available profiles: X={x_count}, Y={y_count}")

        # Select profiles via GUI
        selected_profiles, x_pros, y_pros, ok_flag = self._run_profile_gui(
            x_count, y_count
        )

        if ok_flag < 0:
            return

        # Extract selected profiles
        profile_pcs = extract_profiles(selected_pc, selected_profiles, s['spacing'])

        if len(profile_pcs) == 0:
            self.get_logger().warn("No profiles extracted!")
            return

        # === Open3D Preview: Show extracted profiles ===
        if s['dbug']:
            mesh = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
            vis_list = [mesh]
            for pc in profile_pcs:
                vis_list.append(pc)
            self.get_logger().info("Showing extracted profiles (close window to continue)...")
            o3d.visualization.draw_geometries(
                vis_list,
                window_name='08b_Object Profiles',
                point_show_normal=True
            )

        # Generate targets for each profile
        all_eef_targets = []

        for i, profile_pc in enumerate(profile_pcs):
            self.get_logger().info(
                f"Generating targets for profile {i + 1}/{len(profile_pcs)}"
            )

            # Generate coordinates
            cam_coords, world_coords = generate_coordinates(
                profile_pc, self.tf_buffer, node=self
            )

            # Filter coordinates
            cam_coords, world_coords = filter_coordinates(
                cam_coords, world_coords,
                s['spacing'], s['tgt_final_trim']
            )

            if len(world_coords) == 0:
                continue

            # Generate final EEF targets
            cam_targets, eef_targets, _ = generate_final_coordinates(
                world_coords, s['z_offset'], self.tf_buffer,
                eef_link='Link_6', coord_skip=s['coord_skip'], node=self
            )

            # Reverse if configured
            if s['tgt_reverse']:
                eef_targets = eef_targets[::-1]
                cam_targets = cam_targets[::-1]

            all_eef_targets.extend(eef_targets)

            # Preview targets in RViz
            if s['tgt_preview']:
                tf_utils.publish_coordinates(
                    self.tf_static_broadcaster,
                    cam_targets, 'world', 'camera_target',
                    static=True, clock=self.get_clock()
                )

        if len(all_eef_targets) == 0:
            self.get_logger().error("No valid targets generated!")
            return

        self.get_logger().info(f"Total EEF targets: {len(all_eef_targets)}")

        # Execute targets
        success, total = self.moveit.execute_targets(
            all_eef_targets,
            motion_delay=s['tgt_motion_delay']
        )

        # Save targets if configured
        if s['tgt_save']:
            self._save_targets(np.array(all_eef_targets))

    def _run_auto_mode(self):
        """Execute automatic scanning mode."""
        s = self.config.settings

        # Auto GUI for setup
        result = self._run_auto_gui(s['manual_offset'], s['tgt_save'])
        pose_idx, manual_offset, cluster_idx, motion_dir, tgt_save, iterations, hide_prev, exit_flag = result

        if exit_flag:
            return

        s['manual_offset'] = manual_offset
        s['tgt_save'] = tgt_save

        # Move to initial pose
        self._move_to_initial_pose(pose_idx)
        time.sleep(1.0)

        for iteration in range(iterations):
            self.get_logger().info(f"=== Iteration {iteration + 1}/{iterations} ===")

            # Capture point cloud
            downpcd = self.pc_processor.load_point_cloud(
                samples=s['samples'],
                offset_y=s['offset_y'],
                offset_z=s['offset_z'],
                manual_offset=s['manual_offset'],
                spacing=s['spacing'],
                trim_base=s['trim_base'],
                tf_buffer=self.tf_buffer,
                hide_prev=hide_prev and iteration > 0,
                dbug=s['dbug']
            )

            if len(downpcd.points) == 0:
                self.get_logger().warn(f"Empty point cloud at iteration {iteration + 1}")
                continue

            # Cluster
            clouds = cluster_point_cloud(
                downpcd, eps=s['eps'], min_points=s['min_points'],
                discard_threshold=s['cluster_discard'],
                logger=self.get_logger()
            )

            # Auto-select cluster
            if cluster_idx < len(clouds):
                selected_pc = clouds[cluster_idx]
            else:
                selected_pc = clouds[0]

            if s['cluster_trim'] > 0:
                selected_pc = trim_cluster(selected_pc, s['cluster_trim'])

            if len(selected_pc.normals) == 0:
                selected_pc.estimate_normals(
                    search_param=o3d.geometry.KDTreeSearchParamHybrid(
                        radius=0.02, max_nn=30
                    )
                )
                selected_pc.orient_normals_towards_camera_location(np.array([0., 0., 0.]))

            # Auto profile: use curvature-based direction detection
            x_count, y_count, _, _ = get_profile_counts(selected_pc, s['spacing'])

            if motion_dir == 0:  # Auto: detect curvature direction
                normals = np.asarray(selected_pc.normals)
                if len(normals) > 0:
                    xs, ys = get_xy_angles_from_pc(normals)
                    x_dev = np.std(xs)
                    y_dev = np.std(ys)
                    self.get_logger().info(
                        f"Curvature detection: X_std={x_dev:.2f}, Y_std={y_dev:.2f}"
                    )
                    if x_dev <= y_dev:
                        # Object curves around X → profile in Y direction
                        profiles = [[1, y_count // 2]]
                    else:
                        # Object curves around Y → profile in X direction
                        profiles = [[0, x_count // 2]]
                else:
                    profiles = [[0, x_count // 2], [1, y_count // 2]]
            elif motion_dir == 2:  # X
                profiles = [[0, x_count // 2]]
            elif motion_dir == 3:  # Y
                profiles = [[1, y_count // 2]]
            else:
                profiles = [[0, x_count // 2]]

            profile_pcs = extract_profiles(selected_pc, profiles, s['spacing'])

            # Generate and execute targets
            for profile_pc in profile_pcs:
                cam_coords, world_coords = generate_coordinates(
                    profile_pc, self.tf_buffer
                )
                cam_coords, world_coords = filter_coordinates(
                    cam_coords, world_coords, s['spacing'], s['tgt_final_trim']
                )

                if len(world_coords) == 0:
                    continue

                _, eef_targets, _ = generate_final_coordinates(
                    world_coords, s['z_offset'], self.tf_buffer,
                    coord_skip=s['coord_skip']
                )

                if s['tgt_reverse']:
                    eef_targets = eef_targets[::-1]

                self.moveit.execute_targets(
                    eef_targets, motion_delay=s['tgt_motion_delay']
                )

                if s['tgt_save']:
                    self._save_targets(np.array(eef_targets))

    def _run_replay_mode(self):
        """Execute replay mode from saved targets."""
        s = self.config.settings
        default_path = os.path.expanduser('~/jaka_targets/')

        result = self._run_replay_gui(
            default_path, 0.0, 0.0, 0.0, s['tgt_save']
        )
        file_path, x_offset, y_offset, z_offset, tgt_save, ok_flag = result

        if ok_flag < 0:
            return

        if ok_flag == 2:
            # Set initial pose
            self._move_to_initial_pose(0)
            return

        try:
            targets = np.load(file_path, allow_pickle=True)
            self.get_logger().info(f"Loaded {len(targets)} targets from {file_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to load targets: {e}")
            return

        # Apply offsets
        targets[:, 0] += x_offset
        targets[:, 1] += y_offset
        targets[:, 2] += z_offset

        # Preview
        if s['tgt_preview']:
            tf_utils.publish_coordinates(
                self.tf_static_broadcaster,
                targets, 'base_link', 'replay_target',
                static=True, clock=self.get_clock()
            )

        if ok_flag == 0:
            self.get_logger().info("Preview only — not executing")
            return

        # Execute
        self.moveit.execute_targets(
            targets.tolist(), motion_delay=s['tgt_motion_delay']
        )

    def _run_settings(self):
        """Open settings GUI and update config."""
        settings_list = self.config.get_settings_list()
        result = self._call_gui('Settings_gui.py', settings_list)

        if result[0] == 1:  # Save
            self.config.update_settings_from_list(result[1:])
            self.config.save()
            self.get_logger().info("Settings saved")

    # ======================== Initial Pose ========================

    def _move_to_initial_pose(self, pose_idx):
        """Move robot to an initial scanning pose."""
        pose_names = self.config.get_pose_names()

        if pose_idx >= len(pose_names):
            # Map out-of-range GUI indices to last pose (current_pose)
            pose_idx = len(pose_names) - 1

        name = pose_names[pose_idx]
        joints = self.config.get_pose(name)

        if joints is None:
            self.get_logger().info(f"Using current pose ('{name}')")
            return

        self.get_logger().info(f"Moving to initial pose: {name}")
        self.moveit.go_to_joint_state(joints)

    # ======================== Target Saving ========================

    def _save_targets(self, targets):
        """Save target coordinates to .npy file."""
        save_dir = os.path.expanduser('~/jaka_targets/')
        os.makedirs(save_dir, exist_ok=True)

        # Generate unique filename
        idx = 0
        while True:
            path = os.path.join(save_dir, f'targets_{idx:03d}.npy')
            if not os.path.exists(path):
                break
            idx += 1

        np.save(path, targets)
        self.get_logger().info(f"Targets saved to {path}")

    # ======================== GUI Wrappers ========================

    def _call_gui(self, gui_script, data_in):
        """Call a PySimpleGUI script via subprocess."""
        gui_path = os.path.join(self.gui_dir, gui_script)

        try:
            result = subprocess.run(
                [sys.executable, gui_path],
                capture_output=True, text=True, check=True,
                input=repr(data_in), cwd=self.gui_dir
            )
            return literal_eval(result.stdout.strip())
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"GUI error ({gui_script}): {e.stderr}")
            return None
        except Exception as e:
            self.get_logger().error(f"GUI error ({gui_script}): {e}")
            return None

    def _run_front_gui(self):
        """Run the front/main menu GUI."""
        result = self._call_gui('Front_gui.py', 0)
        return result if result is not None else -1

    def _run_manual_gui(self, manual_offset, tgt_save):
        """Run the manual mode GUI with pose selection images."""
        from PIL import Image as PIL_img
        from io import BytesIO
        import glob

        dat = []
        img_dir = os.path.join(self.gui_dir, 'VI_appdata', 'Robo_object_positions')
        for f in sorted(glob.iglob(os.path.join(img_dir, '*'))):
            try:
                im = PIL_img.open(f)
                buf = BytesIO()
                im.save(buf, format="PNG")
                dat.append(buf.getvalue())
            except Exception:
                pass

        if not dat:
            self.get_logger().warn("No pose images found, using defaults")
            return 0, manual_offset, tgt_save, 0

        result = self._call_gui('Manual_gui.py', [dat, manual_offset, tgt_save])
        if result is None:
            return 0, manual_offset, tgt_save, 1
        return result[0], result[1], result[2], result[3]

    def _run_auto_gui(self, manual_offset, tgt_save):
        """Run the auto mode GUI."""
        from PIL import Image as PIL_img
        from io import BytesIO
        import glob

        dat = []
        img_dir = os.path.join(self.gui_dir, 'VI_appdata', 'Robo_object_positions')
        for f in sorted(glob.iglob(os.path.join(img_dir, '*'))):
            try:
                im = PIL_img.open(f)
                buf = BytesIO()
                im.save(buf, format="PNG")
                dat.append(buf.getvalue())
            except Exception:
                pass

        if not dat:
            return 0, manual_offset, 0, 0, tgt_save, 1, False, 1

        result = self._call_gui('Auto_gui.py', [dat, manual_offset, tgt_save])
        if result is None:
            return 0, manual_offset, 0, 0, tgt_save, 1, False, 1
        return result

    def _run_cluster_gui(self, clouds):
        """Run cluster selection GUI."""
        dat = []
        for cloud in clouds:
            try:
                img = fetch_cloud_image(cloud, rx=120, rz=180)
                dat.append(img)
            except Exception:
                pass

        if not dat:
            return [0]

        result = self._call_gui('Cluster_selection_gui.py', dat)
        return result if result is not None else [0]

    def _run_profile_gui(self, x_profiles, y_profiles, prev_selected=None):
        """Run profile selection GUI."""
        if prev_selected is None:
            prev_selected = []
        result = self._call_gui(
            'Profile_selection_gui.py',
            [x_profiles, y_profiles, prev_selected]
        )
        if result is None:
            return [], [], [], -1
        return result[0], result[1], result[2], result[3]

    def _run_replay_gui(self, file_path, x_off, y_off, z_off, tgt_save):
        """Run replay mode GUI."""
        result = self._call_gui(
            'Replay_gui.py',
            [file_path, x_off, y_off, z_off, tgt_save]
        )
        if result is None:
            return file_path, 0, 0, 0, tgt_save, -1
        return result


def main(args=None):
    """Entry point for the auto planner node."""
    rclpy.init(args=args)

    node = AutoPlannerNode()

    try:
        # Wait for TF to be ready
        time.sleep(2.0)
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

"""
Point cloud processing module for JAKA Visual Inspection.

Handles depth/color frame capture, point cloud generation,
filtering, and sampling using Open3D.
"""

import numpy as np
import open3d as o3d
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

from . import tf_utils


class PointCloudProcessor:
    """Handles depth camera frame capture and point cloud generation."""

    def __init__(self, node):
        """
        Args:
            node: ROS2 node (for subscriptions and logging)
        """
        self.node = node
        self.bridge = CvBridge()
        self.logger = node.get_logger()

    def grab_frame(self, color_topic='/camera/color/image_raw',
                   depth_topic='/camera/depth/image_raw', timeout_sec=10.0):
        """
        Grab one color and one depth frame from the camera topics.

        Returns:
            color_frame: numpy array (H, W, 3) RGB
            depth_frame: numpy array (H, W) uint16
        """
        self.logger.info("Waiting for color frame...")
        color_msg = self._wait_for_message(color_topic, Image, timeout_sec)
        color_frame = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='rgb8')

        self.logger.info("Waiting for depth frame...")
        depth_msg = self._wait_for_message(depth_topic, Image, timeout_sec)
        depth_frame = self.bridge.imgmsg_to_cv2(depth_msg)

        return color_frame, depth_frame

    def get_cam_param(self, info_topic='/camera/depth/camera_info', timeout_sec=10.0):
        """
        Get camera intrinsic parameters from CameraInfo topic.

        Returns:
            w, h, fx, fy, cx, cy
        """
        self.logger.info("Waiting for camera info...")
        info_msg = self._wait_for_message(info_topic, CameraInfo, timeout_sec)

        w = info_msg.width
        h = info_msg.height
        fx = info_msg.k[0]
        fy = info_msg.k[4]
        cx = info_msg.k[2]
        cy = info_msg.k[5]

        return w, h, fx, fy, cx, cy

    def generate_point_cloud(self, color_frame, depth_frame, trim_base,
                             depth_scale=1000.0, depth_trunc=3.0,
                             cam_info_topic='/camera/depth/camera_info',
                             dbug=False):
        """
        Generate an Open3D point cloud from color and depth frames.

        Args:
            color_frame: RGB image (H, W, 3)
            depth_frame: depth image (H, W)
            trim_base: trim distance from base to remove background (meters)
            depth_scale: depth scaling factor (1000 for mm -> m)
            depth_trunc: max depth truncation (meters)
            dbug: if True, show debug visualizations

        Returns:
            pcd: Open3D PointCloud
        """
        w, h, fx, fy, cx, cy = self.get_cam_param(cam_info_topic)

        # Create Open3D camera intrinsics
        intrinsic = o3d.camera.PinholeCameraIntrinsic(w, h, fx, fy, cx, cy)

        # Identity extrinsic (camera frame)
        extrinsic = np.eye(4)

        # Create RGBD image
        # Handle Gazebo simulation (float32 meters) vs Real camera (uint16 millimeters)
        depth_array = np.asarray(depth_frame)
        
        self.logger.info(f"Depth raw shape: {depth_array.shape}, dtype: {depth_array.dtype}, min: {np.min(depth_array)}, max: {np.max(depth_array)}")

        if depth_array.dtype == np.float32 or depth_array.dtype == np.float64:
            # Replace NaNs and Infs with 0
            depth_array = np.nan_to_num(depth_array, posinf=0.0, neginf=0.0)
            # Convert meters to millimeters and cast to uint16
            depth_array = (depth_array * 1000.0).astype(np.uint16)
        else:
            depth_array = depth_array.astype(np.uint16)
            
        self.logger.info(f"Depth uint16 min: {np.min(depth_array)}, max: {np.max(depth_array)}")

        color_raw = o3d.geometry.Image(np.asarray(color_frame))
        depth_raw = o3d.geometry.Image(depth_array)

        rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
            color_raw, depth_raw,
            depth_scale=depth_scale,
            depth_trunc=depth_trunc,
            convert_rgb_to_intensity=False
        )

        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
            rgbd_image, intrinsic, extrinsic
        )

        if len(pcd.points) == 0:
            self.logger.warn(f"Empty point cloud generated! Depth_trunc was {depth_trunc}m.")
            return pcd

        # Crop to filter background
        bbox = pcd.get_axis_aligned_bounding_box()
        min_b = bbox.min_bound
        max_b = bbox.max_bound

        crop_box = o3d.geometry.AxisAlignedBoundingBox(
            min_bound=(min_b[0], min_b[1], min_b[2]),
            max_bound=(max_b[0], max_b[1], max_b[2] - trim_base)
        )
        pcd = pcd.crop(crop_box)

        return pcd

    def get_filtered_point_cloud(self, offset_y, offset_z, manual_offset,
                                 trim_base, tf_buffer, dbug=False):
        """
        Capture a single frame and generate a filtered point cloud.

        Args:
            offset_y: Y-axis crop offset (meters)
            offset_z: Z-axis crop offset (meters)
            manual_offset: manual horizontal offset (meters)
            trim_base: background trimming distance (meters)
            tf_buffer: tf2_ros.Buffer for transform lookups
            dbug: debug visualization flag

        Returns:
            pcd: filtered Open3D PointCloud
        """
        # Get camera-to-robot-base depth
        self.logger.info("Looking up TF: camera_depth_optical_frame → world ...")
        cam_to_base = tf_utils.fetch_transform(
            tf_buffer, 'camera_depth_optical_frame', 'world', quat=False, node=self.node
        )
        cam_robo_depth = abs(cam_to_base[2])
        self.logger.info(f"Camera-to-base depth: {cam_robo_depth:.3f} m")

        # Grab frame and generate point cloud
        color_frame, depth_frame = self.grab_frame()
        depth_trunc = manual_offset + cam_robo_depth - trim_base

        pcd = self.generate_point_cloud(
            color_frame, depth_frame, trim_base
        )

        if len(pcd.points) == 0:
            return pcd

        # Additional cropping to remove robot shadow
        bbox = pcd.get_axis_aligned_bounding_box()
        min_b = bbox.min_bound
        max_b = bbox.max_bound

        # Compute safe crop bounds (ensure min < max on every axis)
        crop_min = (min_b[0] + 0.01, min_b[1] + 0.01, min_b[2])
        crop_max = (max_b[0] - 0.01, max_b[1] - offset_y, max_b[2] - offset_z)

        # Validate: if any axis has min >= max, skip cropping
        if all(cmin < cmax for cmin, cmax in zip(crop_min, crop_max)):
            crop_box = o3d.geometry.AxisAlignedBoundingBox(
                min_bound=crop_min, max_bound=crop_max
            )
            pcd = pcd.crop(crop_box)
        else:
            self.logger.warn("Crop bounds inverted — skipping additional filtering")

        return pcd

    def load_point_cloud(self, samples, offset_y, offset_z, manual_offset,
                         spacing, trim_base, tf_buffer,
                         hide_prev=False, dbug=False):
        """
        Sample multiple point clouds and select the best one.

        Takes multiple samples and selects the one with the most representative
        number of points (filters outlier captures).

        Args:
            samples: number of point cloud samples to take
            offset_y, offset_z: crop offsets
            manual_offset: horizontal offset between robot base and object
            spacing: point cloud spacing/resolution
            trim_base: background trimming distance
            tf_buffer: tf2_ros.Buffer
            hide_prev: hide preview visualizations
            dbug: debug mode

        Returns:
            downpcd: best filtered Open3D PointCloud
        """
        pts_cloud_tot = []
        pts_tot = []

        for i in range(samples):
            new_pt = self.get_filtered_point_cloud(
                offset_y, offset_z, manual_offset, trim_base, tf_buffer, dbug
            )
            pts_cloud_tot.append(new_pt)
            pts_tot.append(len(np.asarray(new_pt.points)))

        if len(pts_cloud_tot) == 0:
            self.logger.error("No point clouds captured!")
            return o3d.geometry.PointCloud()

        # Majority vote: combine all frames with similar point counts
        pts_tot = np.array(pts_tot)
        downpcd = o3d.geometry.PointCloud()

        if len(pts_tot) > 1:
            # Bin point counts and select the most common bin
            num_bins = max(int(np.ceil(samples / 3)), 2)
            bins = np.linspace(np.min(pts_tot), np.max(pts_tot), num_bins)
            digitized = np.digitize(pts_tot, bins)
            uni, count = np.unique(digitized, return_counts=True)
            unique_val = uni[np.argmax(count)]
            idx_list = np.where(digitized == unique_val)[0]

            # Combine all matching point clouds
            for idx in idx_list:
                downpcd += pts_cloud_tot[idx]
            self.logger.info(
                f"Majority vote: combined {len(idx_list)}/{samples} frames"
            )
        else:
            downpcd = pts_cloud_tot[0]

        if len(downpcd.points) == 0:
            return downpcd

        # Hidden point removal (remove occluded/back-facing points)
        diameter = np.linalg.norm(
            np.asarray(downpcd.get_max_bound()) -
            np.asarray(downpcd.get_min_bound())
        )
        cam_hpr = [0, 0, -diameter]
        radius_hpr = diameter * 100
        _, pt_map = downpcd.hidden_point_removal(cam_hpr, radius_hpr)
        downpcd = downpcd.select_by_index(pt_map)
        self.logger.info(
            f"After hidden point removal: {len(downpcd.points)} points"
        )

        # Downsample if spacing is specified
        if spacing > 0 and len(downpcd.points) > 0:
            downpcd = downpcd.voxel_down_sample(voxel_size=spacing)

        # Estimate normals
        if len(downpcd.points) > 0:
            downpcd.estimate_normals(
                search_param=o3d.geometry.KDTreeSearchParamHybrid(
                    radius=0.06, max_nn=30
                )
            )
            downpcd.orient_normals_towards_camera_location(np.array([0., 0., 0.]))

        self.logger.info(f"Final point cloud: {len(downpcd.points)} points")

        if not hide_prev and not dbug:
            pass  # Preview handled by GUI

        return downpcd

    def _wait_for_message(self, topic, msg_type, timeout_sec=10.0):
        """Wait for a single message on a topic (ROS2 equivalent of rospy.wait_for_message)."""
        received = {'msg': None}

        def callback(msg):
            if received['msg'] is None:
                received['msg'] = msg

        sub = self.node.create_subscription(msg_type, topic, callback, 1)

        start_time = self.node.get_clock().now()
        timeout = rclpy.duration.Duration(seconds=timeout_sec)

        while received['msg'] is None:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            elapsed = self.node.get_clock().now() - start_time
            if elapsed > timeout:
                self.node.destroy_subscription(sub)
                raise TimeoutError(f"Timeout waiting for message on {topic}")

        self.node.destroy_subscription(sub)
        return received['msg']

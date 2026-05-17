"""
Target generation module for JAKA Visual Inspection.

Generates EEF target poses from point cloud profiles using
surface normals and camera-to-EEF offset transforms.
Replaces PyKDL with numpy homogeneous transforms.
"""

import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy import spatial

from . import tf_utils


def generate_coordinates(point_cloud, tf_buffer, node=None):
    """
    Generate camera-frame and world-frame coordinates for each point
    in a point cloud, taking surface normals into account.

    Args:
        point_cloud: Open3D PointCloud with normals estimated
        tf_buffer: tf2_ros.Buffer for transform lookups

    Returns:
        camera_coords: Nx7 array [x, y, z, qx, qy, qz, qw] in camera frame
        world_coords:  Nx7 array [x, y, z, qx, qy, qz, qw] in world frame
    """
    pts = np.asarray(point_cloud.points)
    normals = np.asarray(point_cloud.normals)

    if len(pts) == 0:
        empty = np.array([]).reshape(0, 7)
        return empty, empty

    # Get world→camera transform as 4x4 matrix
    world_to_cam = tf_utils.fetch_transform(
        tf_buffer, 'world', 'camera_depth_optical_frame', quat=True, node=node
    )
    T_world_cam = tf_utils.pose_to_matrix(*world_to_cam)

    # Flip frame: rotate 180° around Y to match camera orientation
    # Reference: KDL_flip_frame = PyKDL.Frame(PyKDL.Rotation.RPY(0, np.pi, 0))
    # PyKDL RPY is fixed-axis XYZ (same as scipy 'xyz' intrinsic)
    T_flip = tf_utils.rpy_to_matrix(0, np.pi, 0)

    camera_coords = []
    world_coords = []

    for i in range(len(pts)):
        # Get rotation from surface normal
        rotation, _, _, _ = tf_utils.get_rotation_from_normal(-normals[i])
        rot_mat = rotation.as_matrix()

        # Build original plane frame in camera coordinates
        T_plane = np.eye(4)
        T_plane[:3, :3] = rot_mat
        T_plane[0, 3] = pts[i][0]
        T_plane[1, 3] = pts[i][1]
        T_plane[2, 3] = pts[i][2]

        # Camera→plane with flip
        T_cam_plane = T_plane @ T_flip

        # Extract camera-frame coordinates
        cam_pose = tf_utils.matrix_to_pose(T_cam_plane)
        camera_coords.append(cam_pose)

        # World→plane = World→Camera × Camera→Plane
        T_world_plane = T_world_cam @ T_cam_plane
        world_pose = tf_utils.matrix_to_pose(T_world_plane)
        world_coords.append(world_pose)

    return np.array(camera_coords), np.array(world_coords)


def generate_final_coordinates(world_coords, z_offset, tf_buffer,
                               eef_link='Link_6', coord_skip=3, node=None):
    """
    Generate final EEF target coordinates from world coordinates.

    Applies z_offset (camera-object distance) and transforms from
    camera targets to EEF targets.

    Args:
        world_coords: Nx7 array in world frame
        z_offset: distance from camera to object surface (meters)
        tf_buffer: tf2_ros.Buffer
        eef_link: EEF link name (default 'Link_6' for JAKA ZU5)
        coord_skip: take every Nth coordinate to reduce density

    Returns:
        cam_targets: list of [x,y,z,qx,qy,qz,qw] for camera targets
        eef_targets: list of [x,y,z,qx,qy,qz,qw] for EEF targets
        wc_filtered: filtered world coordinates
    """
    cam_targets = []
    eef_targets = []
    wc_filtered = []

    # Get EEF-to-camera transform (fixed for the robot)
    eef_to_cam = tf_utils.fetch_transform(
        tf_buffer, 'camera_depth_optical_frame', eef_link, quat=True, node=node
    )
    T_eef_cam = tf_utils.pose_to_matrix(*eef_to_cam)

    # Flip frame for camera orientation
    # Reference: KDL_flip_frame = PyKDL.Frame(PyKDL.Rotation.RPY(np.pi, 0., np.pi))
    # PyKDL RPY is fixed-axis XYZ (same as scipy 'xyz' intrinsic)
    T_flip = tf_utils.rpy_to_matrix(np.pi, 0, np.pi)

    for idx in range(0, len(world_coords), coord_skip):
        wc = world_coords[idx]
        wc_filtered.append(wc)

        # Build world→plane matrix from world coordinates
        T_world_plane = tf_utils.pose_to_matrix(*wc)

        # Apply z_offset along the plane's local Z axis (away from surface)
        # Reference: trans_x,trans_y,trans_z = KDL_original_plane_frame * PyKDL.Vector(0, 0, z_offset)
        # PyKDL Frame * Vector applies the frame's rotation+translation to the vector
        # This moves the point along the frame's local Z (positive = away from surface)
        offset_local = np.array([0, 0, z_offset, 1])
        offset_world = T_world_plane @ offset_local
        T_world_plane[0, 3] = offset_world[0]
        T_world_plane[1, 3] = offset_world[1]
        T_world_plane[2, 3] = offset_world[2]

        # Camera target = plane frame × flip
        T_cam_target = T_world_plane @ T_flip
        cam_pose = tf_utils.matrix_to_pose(T_cam_target)
        cam_targets.append(list(cam_pose))

        # EEF target = plane frame × flip × eef-camera transform
        T_eef_target = T_world_plane @ T_flip @ T_eef_cam
        eef_pose = tf_utils.matrix_to_pose(T_eef_target)
        eef_targets.append(list(eef_pose))

    return cam_targets, eef_targets, np.array(wc_filtered)


def filter_coordinates(cam_coords, world_coords, spacing,
                       tgt_final_trim=0.0, remove_close=True):
    """
    Filter generated coordinates to remove unreachable or duplicate targets.

    Args:
        cam_coords: Nx7 camera frame coordinates
        world_coords: Nx7 world frame coordinates
        spacing: point cloud spacing (for close-target detection)
        tgt_final_trim: Z-threshold to remove low targets
        remove_close: remove targets that are too close together

    Returns:
        cam_filtered: filtered camera coordinates
        wc_filtered: filtered world coordinates
    """
    if len(world_coords) == 0:
        return cam_coords, world_coords

    cam_t = np.array(cam_coords)
    wc_t = np.array(world_coords)

    # Remove targets below Z threshold
    if tgt_final_trim > 0:
        ind = np.where(wc_t[:, 2] >= tgt_final_trim)[0]
        wc_t = wc_t[ind]
        cam_t = cam_t[ind]
        removed = len(world_coords) - len(wc_t)
        if removed > 0:
            print(f"Removed {removed} targets below Z threshold ({tgt_final_trim}m)")

    # Remove targets that are too close together
    if remove_close and len(wc_t) > 3:
        i = 1
        while i < len(wc_t):
            dist = spatial.distance.euclidean(wc_t[i - 1][:3], wc_t[i][:3])
            if dist < 2 * spacing:
                wc_t = np.delete(wc_t, i, axis=0)
                cam_t = np.delete(cam_t, i, axis=0)
            else:
                i += 1

    return cam_t, wc_t

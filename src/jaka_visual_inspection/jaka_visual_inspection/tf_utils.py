"""
TF2 and transform utility functions for JAKA Visual Inspection.

Replaces PyKDL frame math from the reference project with
numpy 4x4 homogeneous transforms and scipy rotations.
"""

import numpy as np
from scipy.spatial.transform import Rotation as R
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
import tf2_ros
import geometry_msgs.msg
import math
import time


def fetch_transform(tf_buffer, parent_frame, child_frame, quat=False, timeout_sec=2.0, node=None):
    """
    Look up a transform from parent_frame to child_frame using TF2.

    Args:
        tf_buffer: tf2_ros.Buffer instance
        parent_frame: parent frame id (e.g., 'world')
        child_frame: child frame id (e.g., 'camera_depth_optical_frame')
        quat: if True, return quaternion; else return euler angles
        timeout_sec: max wait time for transform

    Returns:
        If quat=False: (x, y, z, roll, pitch, yaw) in radians
        If quat=True:  (x, y, z, qx, qy, qz, qw)
    """
    max_retries = 10
    for attempt in range(max_retries):
        try:
            trans = tf_buffer.lookup_transform(
                parent_frame,
                child_frame,
                Time()
            )
            t = trans.transform.translation
            r = trans.transform.rotation

            if quat:
                return (t.x, t.y, t.z, r.x, r.y, r.z, r.w)
            else:
                rot = R.from_quat([r.x, r.y, r.z, r.w])
                rpy = rot.as_euler('XYZ', degrees=False)
                return (t.x, t.y, t.z, rpy[0], rpy[1], rpy[2])
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            if node:
                rclpy.spin_once(node, timeout_sec=0.2)
            else:
                time.sleep(0.2)
            continue
    raise RuntimeError(
        f"Failed to lookup transform {parent_frame} → {child_frame} "
        f"after {max_retries} attempts"
    )


def publish_coordinates(broadcaster, coordinates, parent_frame, child_prefix,
                        static=False, clock=None):
    """
    Publish transform frames for a set of coordinates.

    Args:
        broadcaster: tf2_ros.TransformBroadcaster or StaticTransformBroadcaster
        coordinates: Nx7 array [[x, y, z, qx, qy, qz, qw], ...]
        parent_frame: parent frame id
        child_prefix: prefix for child frame names
        static: whether to use static broadcaster
        clock: rclpy.clock.Clock for timestamps
    """
    for index in range(len(coordinates)):
        t = geometry_msgs.msg.TransformStamped()
        if clock is not None:
            t.header.stamp = clock.now().to_msg()
        t.header.frame_id = parent_frame

        suffix = "_static_" if static else "_"
        t.child_frame_id = f"{child_prefix}{suffix}{index}"

        t.transform.translation.x = float(coordinates[index][0])
        t.transform.translation.y = float(coordinates[index][1])
        t.transform.translation.z = float(coordinates[index][2])
        t.transform.rotation.x = float(coordinates[index][3])
        t.transform.rotation.y = float(coordinates[index][4])
        t.transform.rotation.z = float(coordinates[index][5])
        t.transform.rotation.w = float(coordinates[index][6])

        broadcaster.sendTransform(t)
        time.sleep(0.001)


def quat_to_matrix(qx, qy, qz, qw):
    """Convert quaternion to 4x4 homogeneous transformation matrix (rotation only)."""
    rot = R.from_quat([qx, qy, qz, qw])
    mat = np.eye(4)
    mat[:3, :3] = rot.as_matrix()
    return mat


def pose_to_matrix(x, y, z, qx, qy, qz, qw):
    """Convert pose (position + quaternion) to 4x4 homogeneous matrix."""
    mat = quat_to_matrix(qx, qy, qz, qw)
    mat[0, 3] = x
    mat[1, 3] = y
    mat[2, 3] = z
    return mat


def matrix_to_pose(mat):
    """Convert 4x4 homogeneous matrix to (x, y, z, qx, qy, qz, qw)."""
    x, y, z = mat[0, 3], mat[1, 3], mat[2, 3]
    rot = R.from_matrix(mat[:3, :3])
    qx, qy, qz, qw = rot.as_quat()
    return (x, y, z, qx, qy, qz, qw)


def rpy_to_matrix(roll, pitch, yaw):
    """
    Convert RPY angles to 4x4 homogeneous matrix (rotation only).

    Matches PyKDL.Rotation.RPY(roll, pitch, yaw) convention:
    PyKDL RPY applies rotations as: first yaw (Z), then pitch (Y), then roll (X)
    in the fixed/world frame. This is equivalent to scipy 'sxyz' or extrinsic XYZ.
    scipy's 'XYZ' uppercase = extrinsic (fixed-axis) which matches PyKDL.
    """
    rot = R.from_euler('XYZ', [roll, pitch, yaw])
    mat = np.eye(4)
    mat[:3, :3] = rot.as_matrix()
    return mat


def get_rotation_from_normal(normal_vector):
    """
    Compute rotation matrix from a surface normal vector.
    This aligns the Z-axis with the negative normal (camera facing the surface).

    Args:
        normal_vector: 3D normal vector

    Returns:
        rotation: scipy Rotation object
        angle_x, angle_y, angle_z: angles between normal and X/Y/Z axes
    """
    v1 = normal_vector.copy()
    if np.allclose(v1, [0., 0., 1.]):
        v1 = np.array([0, 0.000001, 0.999999])

    vec_x = np.array([1., 0., 0.])
    vec_y = np.array([0., 1., 0.])
    vec_z = np.array([0., 0., 1.])

    vec1 = v1 / np.linalg.norm(v1)

    angle_x = math.acos(np.clip(np.dot(vec1, vec_x), -1.0, 1.0))
    angle_y = math.acos(np.clip(np.dot(vec1, vec_y), -1.0, 1.0))

    cross = np.cross(vec1, vec_z)
    cross_norm = np.linalg.norm(cross)
    if cross_norm > 1e-10:
        vector_z = cross / cross_norm
    else:
        vector_z = np.array([1., 0., 0.])

    angle_z = -(math.acos(np.clip(np.dot(vec1, vec_z), -1.0, 1.0)))

    rotation = R.from_rotvec(angle_z * vector_z)

    return rotation, angle_x, angle_y, angle_z


def axis_angle_to_quat(vector, angle):
    """Convert axis-angle to quaternion."""
    x, y, z = vector
    qx = x * np.sin(angle / 2)
    qy = y * np.sin(angle / 2)
    qz = z * np.sin(angle / 2)
    qw = np.cos(angle / 2)
    return qx, qy, qz, qw

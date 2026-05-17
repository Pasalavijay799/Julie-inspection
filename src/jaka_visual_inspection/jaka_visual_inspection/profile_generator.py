"""
Profile generator module for JAKA Visual Inspection.

Extracts cross-section profiles (X/Y slices) from clustered point clouds.
"""

import numpy as np
import open3d as o3d


def generate_bounds(min_bound, max_bound, spacing):
    """
    Generate slicing bounds for profile extraction.

    Args:
        min_bound: minimum coordinate value
        max_bound: maximum coordinate value
        spacing: spacing between slices

    Returns:
        bounds: Nx2 array of [lower_bound, upper_bound] pairs
    """
    bounds = []
    ctr = 0
    curr_b = min_bound

    while curr_b < max_bound - 0.5 * spacing:
        lower_b = min_bound + ctr * spacing + 0.005

        if ctr == 0:
            lower_b = min_bound + ctr * spacing
            curr_b = lower_b + spacing / 2
        else:
            curr_b = lower_b + spacing

        bounds.append([lower_b, curr_b])
        ctr += 1

    return np.array(bounds) if bounds else np.array([]).reshape(0, 2)


def _resample_and_sort_profile(point_cloud, sort_axis, spacing):
    """
    Resample a profile by flattening the near-constant axis and sorting.

    The reference paper flattens the axis with minimum std deviation to its
    mean, then re-downsamples and sorts along the traversal direction.

    Args:
        point_cloud: Open3D PointCloud (raw profile slice)
        sort_axis: axis index to sort along (0=X, 1=Y)
        spacing: voxel size for re-downsampling

    Returns:
        sorted_pc: resampled and sorted Open3D PointCloud
    """
    if len(point_cloud.points) == 0:
        return point_cloud

    pts = np.asarray(point_cloud.points)
    normals = np.asarray(point_cloud.normals) if len(point_cloud.normals) > 0 else None

    # Flatten the axis with minimum std deviation to its mean
    # This makes the profile a clean 1D line through the object
    min_idx = np.argmin(np.std(pts, axis=0))
    pts[:, min_idx] = np.mean(pts, axis=0)[min_idx]

    point_cloud.points = o3d.utility.Vector3dVector(pts)

    # Re-downsample after flattening to remove duplicates
    if spacing > 0:
        point_cloud = point_cloud.voxel_down_sample(voxel_size=spacing)

    # Get updated arrays
    pts = np.asarray(point_cloud.points)
    normals = np.asarray(point_cloud.normals) if len(point_cloud.normals) > 0 else None

    # Sort along the traversal axis
    ind = np.argsort(pts[:, sort_axis])
    pts = pts[ind]
    sorted_pc = o3d.geometry.PointCloud()
    sorted_pc.points = o3d.utility.Vector3dVector(pts)
    if normals is not None and len(normals) == len(ind):
        normals = normals[ind]
        sorted_pc.normals = o3d.utility.Vector3dVector(normals)

    return sorted_pc


def extract_profile_x(point_cloud, profile_index, bounds, spacing,
                      cluster_trim=0.0):
    """
    Extract a cross-section profile in the X direction (slices along Y).

    Args:
        point_cloud: Open3D PointCloud
        profile_index: index into bounds array
        bounds: Nx2 array from generate_bounds
        spacing: slice thickness
        cluster_trim: trim from cluster edges (meters)

    Returns:
        profile_pc: Open3D PointCloud of the extracted profile (sorted)
    """
    if len(bounds) == 0 or profile_index >= len(bounds):
        return point_cloud

    bbox = point_cloud.get_axis_aligned_bounding_box()
    min_b = bbox.min_bound
    max_b = bbox.max_bound

    x_lower = bounds[profile_index][0]
    x_upper = bounds[profile_index][1]

    # X bounds from profile, Y bounds with cluster trim applied
    crop_box = o3d.geometry.AxisAlignedBoundingBox(
        min_bound=(min_b[0] + cluster_trim, x_lower, min_b[2]),
        max_bound=(max_b[0] - cluster_trim, x_upper, max_b[2])
    )

    # Use select_by_index like reference (handles empty results better)
    indices = crop_box.get_point_indices_within_bounding_box(point_cloud.points)

    # If empty, try next adjacent profile
    orig_idx = profile_index
    while len(indices) == 0 and profile_index < len(bounds) - 1:
        profile_index += 1
        x_lower = bounds[profile_index][0]
        x_upper = bounds[profile_index][1]
        crop_box = o3d.geometry.AxisAlignedBoundingBox(
            min_bound=(min_b[0] + cluster_trim, x_lower, min_b[2]),
            max_bound=(max_b[0] - cluster_trim, x_upper, max_b[2])
        )
        indices = crop_box.get_point_indices_within_bounding_box(point_cloud.points)

    if len(indices) == 0:
        return o3d.geometry.PointCloud()

    result = point_cloud.select_by_index(indices)

    # Resample and sort along X axis (traversal direction for X profiles)
    return _resample_and_sort_profile(result, sort_axis=0, spacing=spacing)


def extract_profile_y(point_cloud, profile_index, bounds, spacing,
                      cluster_trim=0.0):
    """
    Extract a cross-section profile in the Y direction (slices along X).

    Args:
        point_cloud: Open3D PointCloud
        profile_index: index into bounds array
        bounds: Nx2 array from generate_bounds
        spacing: slice thickness
        cluster_trim: trim from cluster edges (meters)

    Returns:
        profile_pc: Open3D PointCloud of the extracted profile (sorted)
    """
    if len(bounds) == 0 or profile_index >= len(bounds):
        return point_cloud

    bbox = point_cloud.get_axis_aligned_bounding_box()
    min_b = bbox.min_bound
    max_b = bbox.max_bound

    y_lower = bounds[profile_index][0]
    y_upper = bounds[profile_index][1]

    # Y bounds from profile, X bounds with cluster trim applied
    crop_box = o3d.geometry.AxisAlignedBoundingBox(
        min_bound=(y_lower, min_b[1] + cluster_trim, min_b[2]),
        max_bound=(y_upper, max_b[1] - cluster_trim, max_b[2])
    )

    indices = crop_box.get_point_indices_within_bounding_box(point_cloud.points)

    # If empty, try next adjacent profile
    while len(indices) == 0 and profile_index < len(bounds) - 1:
        profile_index += 1
        y_lower = bounds[profile_index][0]
        y_upper = bounds[profile_index][1]
        crop_box = o3d.geometry.AxisAlignedBoundingBox(
            min_bound=(y_lower, min_b[1] + cluster_trim, min_b[2]),
            max_bound=(y_upper, max_b[1] - cluster_trim, max_b[2])
        )
        indices = crop_box.get_point_indices_within_bounding_box(point_cloud.points)

    if len(indices) == 0:
        return o3d.geometry.PointCloud()

    result = point_cloud.select_by_index(indices)

    # Resample and sort along Y axis (traversal direction for Y profiles)
    return _resample_and_sort_profile(result, sort_axis=1, spacing=spacing)


def get_profile_counts(point_cloud, spacing):
    """
    Calculate the number of X and Y profiles available for a point cloud.

    Note: Following the reference implementation's convention:
    - X profiles = slices along Y axis (bounds generated from Y min/max)
    - Y profiles = slices along X axis (bounds generated from X min/max)

    Args:
        point_cloud: Open3D PointCloud
        spacing: slice spacing

    Returns:
        x_count: number of X profiles
        y_count: number of Y profiles
        x_bounds: bounds for X profiles (generated from Y range)
        y_bounds: bounds for Y profiles (generated from X range)
    """
    if len(point_cloud.points) == 0:
        return 0, 0, np.array([]).reshape(0, 2), np.array([]).reshape(0, 2)

    bbox = point_cloud.get_axis_aligned_bounding_box()
    min_b = bbox.min_bound
    max_b = bbox.max_bound

    # Reference: profiles_X = Bounds_gen(minB_Y, maxB_Y, spacing)
    #            profiles_Y = Bounds_gen(minB_X, maxB_X, spacing)
    x_bounds = generate_bounds(min_b[1], max_b[1], spacing)
    y_bounds = generate_bounds(min_b[0], max_b[0], spacing)

    return len(x_bounds), len(y_bounds), x_bounds, y_bounds


def extract_profiles(point_cloud, selected_profiles, spacing):
    """
    Extract multiple profiles from a point cloud.

    Args:
        point_cloud: Open3D PointCloud
        selected_profiles: list of [mode, *indices] where mode 0/2/4=X, 1/3/5=Y
        spacing: slice spacing

    Returns:
        profile_pcs: list of Open3D PointCloud objects
    """
    _, _, x_bounds, y_bounds = get_profile_counts(point_cloud, spacing)
    profile_pcs = []

    for profile in selected_profiles:
        mode = profile[0]
        indices = profile[1:]

        for idx in indices:
            if mode in [0, 2, 4]:  # X profiles
                if idx < len(x_bounds):
                    pc = extract_profile_x(point_cloud, idx, x_bounds, spacing)
                    if len(pc.points) > 0:
                        profile_pcs.append(pc)
            elif mode in [1, 3, 5]:  # Y profiles
                if idx < len(y_bounds):
                    pc = extract_profile_y(point_cloud, idx, y_bounds, spacing)
                    if len(pc.points) > 0:
                        profile_pcs.append(pc)

    return profile_pcs


def get_xy_angles_from_pc(normals):
    """
    Calculate angles between normals and X/Y axes.

    Args:
        normals: Nx3 array of normal vectors

    Returns:
        xs: angles to X axis (degrees)
        ys: angles to Y axis (degrees)
    """
    vec_x = np.array([1., 0., 0.])
    vec_y = np.array([0., 1., 0.])

    xs = []
    ys = []

    for normal in normals:
        if np.allclose(normal, [0., 0., 1.]):
            normal = np.array([0, 0.000001, 0.999999])

        vec1 = normal / np.linalg.norm(normal)
        angle_x = np.round(np.degrees(np.arccos(np.clip(np.dot(vec1, vec_x), -1.0, 1.0))))
        angle_y = np.round(np.degrees(np.arccos(np.clip(np.dot(vec1, vec_y), -1.0, 1.0))))

        xs.append(angle_x)
        ys.append(angle_y)

    return np.array(xs), np.array(ys)

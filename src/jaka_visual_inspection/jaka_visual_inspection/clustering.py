"""
DBSCAN clustering module for JAKA Visual Inspection.

Clusters point clouds to isolate target objects from background.
"""

import numpy as np
import open3d as o3d
import copy


def cluster_point_cloud(original_pc, eps=0.02, min_points=10,
                        discard_threshold=0, logger=None):
    """
    Cluster a point cloud using DBSCAN.

    Args:
        original_pc: Open3D PointCloud
        eps: DBSCAN epsilon (max distance between neighbors)
        min_points: DBSCAN min points per cluster
        discard_threshold: minimum points to keep a cluster (0 = keep all)
        logger: optional ROS logger

    Returns:
        clouds: list of Open3D PointCloud objects (one per cluster)
    """
    _log = logger.info if logger else print

    if len(original_pc.points) == 0:
        return [original_pc]

    labels = np.array(original_pc.cluster_dbscan(eps=eps, min_points=min_points))
    unique_labels = np.unique(labels)

    clouds = []

    for label in unique_labels:
        if label == -1:
            continue  # Skip noise points

        idx = np.where(labels == label)[0]
        cluster_pcd = original_pc.select_by_index(idx.tolist())

        # Skip clusters that are too small
        if discard_threshold > 0 and len(cluster_pcd.points) < discard_threshold:
            continue

        clouds.append(cluster_pcd)

    if len(clouds) == 0:
        # If no valid clusters, return the original
        clouds.append(original_pc)

    _log(f"Number of clusters: {len(clouds)}")
    return clouds


def trim_cluster(cluster_pc, trim_amount):
    """
    Trim the edges of a cluster point cloud.

    Args:
        cluster_pc: Open3D PointCloud
        trim_amount: amount to trim from edges (meters)

    Returns:
        trimmed: Open3D PointCloud
    """
    if len(cluster_pc.points) == 0 or trim_amount <= 0:
        return cluster_pc

    bbox = cluster_pc.get_axis_aligned_bounding_box()
    min_b = bbox.min_bound
    max_b = bbox.max_bound

    crop_box = o3d.geometry.AxisAlignedBoundingBox(
        min_bound=(min_b[0] + trim_amount, min_b[1] + trim_amount, min_b[2]),
        max_bound=(max_b[0] - trim_amount, max_b[1] - trim_amount, max_b[2])
    )

    return cluster_pc.crop(crop_box)


def fetch_cloud_image(point_cloud, rx=120, ry=0, rz=180):
    """
    Render a point cloud to an image for GUI display.

    Args:
        point_cloud: Open3D PointCloud
        rx, ry, rz: rotation angles for viewing (degrees)

    Returns:
        data: PNG image bytes
    """
    from PIL import Image as PIL_img
    from io import BytesIO

    mesh = o3d.geometry.TriangleMesh.create_coordinate_frame(
        size=0.1, origin=[0, 0, 0]
    )
    mesh_pc = mesh.sample_points_uniformly(
        number_of_points=1000000, use_triangle_normal=False
    )

    tmp_cloud = copy.deepcopy(point_cloud)
    tmp_cloud += mesh_pc

    rot_matrix = point_cloud.get_rotation_matrix_from_xyz(
        (np.radians(rx), np.radians(ry), np.radians(rz))
    )
    tmp_cloud.rotate(rot_matrix, center=(0, 0, 0))

    vis = o3d.visualization.Visualizer()
    vis.create_window(visible=False, width=640, height=480)
    vis.add_geometry(tmp_cloud)
    vis.poll_events()
    vis.update_renderer()
    color = vis.capture_screen_float_buffer(True)
    vis.destroy_window()

    color = (255.0 * np.asarray(color)).astype(np.uint8)

    # Convert to PNG bytes
    im = PIL_img.fromarray(color)
    output_buffer = BytesIO()
    im.save(output_buffer, format="PNG")
    data = output_buffer.getvalue()

    return data


def combine_clusters(clouds, selected_indices):
    """
    Combine selected clusters into a single point cloud.

    Args:
        clouds: list of Open3D PointCloud objects
        selected_indices: list of indices to combine

    Returns:
        combined: Open3D PointCloud
    """
    combined = o3d.geometry.PointCloud()
    for k in selected_indices:
        if k < len(clouds):
            combined += clouds[k]
    return combined

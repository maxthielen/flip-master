import numpy as np
import open3d as o3d
import statistics as st


def filter_out_steel_plate(pcd: o3d.geometry.PointCloud, thickness: float):
    """
    Filters out all points that lay 1 standard deviation from the mode z-axis.
    There could still remain points that are not part of the steel plate, but are around the given mode.
    These points could be removed with outlier removal

    :param pcd: (o3d.geometry.PointCloud) point cloud
    :param thickness: (float) thickness of steel plate
    :return: (o3d.geometry.PointCloud) filtered point cloud
    """
    # here we filter out all points that lay 1 stdev from the mode
    filtered_xyz = np.asarray(pcd.points)
    filtered_z = filtered_xyz[:, 2]
    st_dev = st.stdev(filtered_z)
    filtered_z[filtered_z < thickness - st_dev * 1.1] = 0
    filtered_z[filtered_z > thickness + st_dev * 1.5] = 0

    # We recreate the pcd with the filtered values.
    # This creates a pcd only with the steel plate points at the mode height
    # This also leaves some points which are at the height of the steel plate,
    # but are not part of the plate. These we will remove later with outlier removal.
    pcd.points = o3d.utility.Vector3dVector(filtered_xyz)
    points = np.asarray(pcd.points)
    return pcd.select_by_index(np.where(points[:, 2] > thickness - 2)[0])


def get_bounding_box(pcd: o3d.geometry.PointCloud):
    """
    Returns the bounding box for the given point cloud.
    Sets the min of the bbox to -1 and the max to mode + stdev
    Also enlarges the bbox by around 5%

    :param pcd: (o3d.geometry.PointCloud) point cloud

    :return: (o3d.geometry.AxisAlignedBoundingBox) enlarged bounding box of point cloud
    """

    # Get the bounding box of the steel plate
    aabb = pcd.get_axis_aligned_bounding_box()
    
    # Get the points of the bbox, so we can resize it.
    aabb_points = np.asarray(aabb.get_box_points())

    # Resize the bbox that makes sure we don't miss anything around the edges.
    aabb_points[:, 0] = aabb_points[:, 0] * [0.95, 1.025, 0.95, 0.95, 1.025, 0.95, 1.025, 1.025]
    aabb_points[:, 1] = aabb_points[:, 1] * [0.95, 1.025, 0.95, 0.95, 1.025, 0.95, 1.025, 1.025]
    print(aabb_points)
    
    # Recreate the bbox with the new values
    aabb_points = o3d.utility.Vector3dVector(aabb_points)
    
    aabb.clear()
    return aabb.create_from_points(aabb_points)


def remove_outliers(pcd: o3d.geometry.PointCloud):
    """
    Downsamples the point cloud with o3d voxel_down_sample,
    then uses statistical outlier removal to remove outliers

    :param pcd: (o3d.geometry.PointCloud) point cloud
    :return: (o3d.geometry.PointCloud) point cloud with removed outliers
    """
    down_pcd = pcd.voxel_down_sample(voxel_size=0.1)
    cl, ind = down_pcd.remove_statistical_outlier(nb_neighbors=50, std_ratio=1.0)
    return down_pcd.select_by_index(ind)
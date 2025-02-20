"""
Author: Yixuan Su
Date: 2025/01/04 16:38
File: LCP.py
Description: 
"""
import open3d as o3d
import numpy as np

def compute_curvature_and_normals(pcd, radius):
    """
    计算点云的法线和曲率
    Args:
        pcd: 点云
        radius: 邻域搜索半径
    Returns:
        法线和曲率信息
    """
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=30))
    pcd.orient_normals_consistent_tangent_plane(k=30)
    curvatures = []
    for i in range(len(pcd.points)):
        neighbors = pcd.get_neighbors(i, search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=30))
        normals = np.asarray([pcd.normals[j] for j in neighbors])
        curvatures.append(np.linalg.norm(np.var(normals, axis=0)))
    return np.asarray(curvatures)

def lcp_registration(source, target, voxel_size, radius):
    """
    使用 LCP 方法对点云进行配准
    """
    # 预处理点云（下采样）
    source_down = source.voxel_down_sample(voxel_size)
    target_down = target.voxel_down_sample(voxel_size)

    # 计算法线和曲率
    source_curvature = compute_curvature_and_normals(source_down, radius)
    target_curvature = compute_curvature_and_normals(target_down, radius)

    # 配准（示例中使用普通 Open3D_ICP_Edition，可替换为自定义优化器）
    result = o3d.pipelines.registration.registration_icp(
        source_down, target_down, max_correspondence_distance=voxel_size * 1.5,
        init=np.eye(4),
        estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        criteria=o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=2000)
    )

    return result.transformation

# 加载点云
source_pcd = o3d.io.read_point_cloud("path_to_source_point_cloud.ply")
target_pcd = o3d.io.read_point_cloud("path_to_target_point_cloud.ply")

# 配准
transformation = lcp_registration(source_pcd, target_pcd, voxel_size=0.05, radius=0.1)
print("Transformation Matrix:")
print(transformation)

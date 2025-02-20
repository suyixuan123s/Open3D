"""
Author: Yixuan Su
Date: 2025/02/17 20:03
File: 彩色点云配准.py
Description: 

"""
import copy
import open3d as o3d
import numpy as np

# -------------------读取点云数据--------------------
source = o3d.io.read_point_cloud("../data/frag_115.ply")
target = o3d.io.read_point_cloud("../data/frag_116.ply")


# source = o3d.io.read_point_cloud(r"E:\ABB-Project\ABB_wrs\suyixuan\Point_Cloud_Processing\Cans\data\Meshclone.ply")
# target = o3d.io.read_point_cloud(r"E:\ABB-Project\ABB_wrs\suyixuan\Point_Cloud_Processing\Cans\data\PointCloud.ply")

def check_normals(pcd):
    # 检查是否有法线信息
    return pcd.has_normals()

# 检查源点云和目标点云是否已经包含法线信息
if not check_normals(source):
    print("Source point cloud does not have normals. Estimating normals...")
    source.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.02, max_nn=30))

if not check_normals(target):
    print("Target point cloud does not have normals. Estimating normals...")
    target.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.02, max_nn=30))


# # 估计法线
# source.estimate_normals(
#     o3d.geometry.KDTreeSearchParamHybrid(radius=0.02, max_nn=30))
# target.estimate_normals(
#     o3d.geometry.KDTreeSearchParamHybrid(radius=0.02, max_nn=30))

# 给目标点云添加颜色，如果没有颜色
if not np.any(np.asarray(target.colors)):  # 如果目标点云没有颜色
    target.paint_uniform_color([0.5, 0.5, 0.5])  # 将目标点云涂成灰色



def draw_registration_result_original_color(raw_source, raw_target, transformation):
    source_temp = copy.deepcopy(raw_source)
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, raw_target], window_name="彩色点云配准",
                                      width=1024, height=768,
                                      left=50, top=50,
                                      mesh_show_back_face=False)


# ----------------可视化点云初始位置-----------------
current_transformation = np.identity(4)
draw_registration_result_original_color(source, target, current_transformation)

# point to plane ICP
current_transformation = np.identity(4)
print("2. Point-to-plane ICP registration is applied on original point")
print("   clouds to refine the alignment. Distance threshold 0.02.")
result_icp = o3d.pipelines.registration.registration_icp(
    source, target, 0.02, current_transformation,
    o3d.pipelines.registration.TransformationEstimationPointToPlane())
print(result_icp)
draw_registration_result_original_color(source, target,
                                        result_icp.transformation)
# colored pointcloud registration
# This is implementation of following paper
# J. Park, Q.-Y. Zhou, V. Koltun,
# Colored Point Cloud Registration Revisited, ICCV 2017
voxel_radius = [0.04, 0.02, 0.01]
max_iter = [50, 30, 14]
current_transformation = np.identity(4)
print("3. Colored point cloud registration")
for scale in range(3):
    iters = max_iter[scale]
    radius = voxel_radius[scale]
    print([iters, radius, scale])

    print("3-1. Downsample with a voxel size %.2f" % radius)
    source_down = source.voxel_down_sample(radius)
    target_down = target.voxel_down_sample(radius)

    print("3-2. Estimate normal.")
    source_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius * 2, max_nn=30))
    target_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius * 2, max_nn=30))

    print("3-3. Applying colored point cloud registration")
    result_icp = o3d.pipelines.registration.registration_colored_icp(
        source_down, target_down, radius, current_transformation,
        o3d.pipelines.registration.TransformationEstimationForColoredICP(),
        o3d.pipelines.registration.ICPConvergenceCriteria(relative_fitness=1e-6,
                                                          relative_rmse=1e-6,
                                                          max_iteration=iters))
    current_transformation = result_icp.transformation
    print(result_icp)
draw_registration_result_original_color(source, target,
                                        result_icp.transformation)

"""
Author: Yixuan Su
Date: 2024/12/17 14:32
File: PL-Open3D_ICP_Edition.py
Description: 
"""

import open3d as o3d
import numpy as np

# 加载点云数据
source = o3d.io.read_point_cloud(
    r"/suyixuan/ABB/Pose_Estimation/Task6_Point_Cloud_Segment_and_Analyze/Dataset_Point_Cloud/normalized_realsense_point_cloud.ply")  # 读取源点云
target = o3d.io.read_point_cloud(
    r"/suyixuan/ABB/Pose_Estimation/Task8_STL_To_Point_Cloud/Datasets/stl_to_point_cloud_voxelization_normalized.ply")  # 读取目标点云

# 设置不同颜色以区分点云
source.paint_uniform_color([1, 0, 0])  # 红色
target.paint_uniform_color([0, 1, 0])  # 绿色

# 可视化原始点云（可选）
print("显示源点云...")
o3d.visualization.draw_geometries([source], window_name="source Registration", width=1024, height=768)

print("显示目标点云...")
o3d.visualization.draw_geometries([target], window_name="target Registration", width=1024, height=768)

# 设定初始粗对齐变换矩阵
initial_guess = np.array([[-0.82647682, -0.06199939, 0.55954637, 0.78003854],
                          [-0.56274889, 0.06308269, -0.82421736, 1.22544392],
                          [0.01580328, -0.99608065, -0.08702647, 0.77418869],
                          [0, 0, 0, 1]])

# 计算目标点云法线（PL-Open3D_ICP_Edition 需要法线信息）
target.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

# 使用 PL-Open3D_ICP_Edition 算法进行点云配准
max_correspondence_distance = 0.5  # 最大匹配距离

reg_icp = o3d.pipelines.registration.registration_icp(
    source, target, max_correspondence_distance, init=initial_guess,
    estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPlane(),
    criteria=o3d.pipelines.registration.ICPConvergenceCriteria(
        max_iteration=3000,  # 最大迭代次数
        relative_fitness=1e-6,  # 相对收敛条件
        relative_rmse=1e-6  # 相对均方根误差收敛条件
    )
)

# 打印配准结果
print("Fitness:", reg_icp.fitness)  # 配准度（匹配的点对数量比例，越接近 1 越好）
print("RMSE:", reg_icp.inlier_rmse)  # 配准误差（越小越好）

# 打印最终变换矩阵
print(f"Transformation Matrix:")
print(reg_icp.transformation)  # 最终变换矩阵

# 应用变换到源点云，得到对齐后的点云
source_pcd = source.transform(reg_icp.transformation)

# 可视化对齐后的点云
o3d.visualization.draw_geometries([source_pcd, target], window_name="After PL-Open3D_ICP_Edition Registration",
                                  width=1024, height=768)

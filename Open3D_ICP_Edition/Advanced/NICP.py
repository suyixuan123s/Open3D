"""
Author: Yixuan Su
Date: 2024/12/17 14:46
File: NICP.py
Description: 
"""

import open3d as o3d
import numpy as np


def normalize_point_cloud(point_cloud):
    # 获取点云坐标
    points = np.asarray(point_cloud.points)

    # 计算点云的边界框
    min_coords = np.min(points, axis=0)
    max_coords = np.max(points, axis=0)

    # 计算边界框的尺度（最大值和最小值之间的差距）
    scale = np.max(max_coords - min_coords)

    # 归一化：将点云缩放到 [0, 1] 范围
    normalized_points = (points - min_coords) / scale

    # 更新点云的坐标
    point_cloud.points = o3d.utility.Vector3dVector(normalized_points)

    return point_cloud, scale, min_coords


# def denormalize_transformation(transformation, scale, min_coords):
#     # 反归一化变换矩阵
#     # 恢复到原始尺度
#     transformation[:3, 3] *= scale
#     return transformation

def denormalize_transformation(transformation, scale, min_coords):
    # 创建一个可变的副本，避免直接修改只读数组
    transformation_copy = transformation.copy()  # 创建副本
    transformation_copy[:3, 3] *= scale  # 对副本进行修改
    return transformation_copy


# 设定初始粗对齐变换矩阵
initial_guess = np.array([[-0.82647682, -0.06199939, 0.55954637, 0.78003854],
                          [-0.56274889, 0.06308269, -0.82421736, 1.22544392],
                          [0.01580328, -0.99608065, -0.08702647, 0.77418869],
                          [0, 0, 0, 1]])


def apply_nicp(source, target, max_correspondence_distance=0.05, max_iterations=10000):
    # 归一化源点云和目标点云
    source_normalized, scale_source, min_coords_source = normalize_point_cloud(source)
    target_normalized, scale_target, min_coords_target = normalize_point_cloud(target)

    # 使用标准的ICP进行点云配准
    reg_icp = o3d.pipelines.registration.registration_icp(
        source_normalized, target_normalized, max_correspondence_distance,
        init=initial_guess,  # 初始变换为单位矩阵
        estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        criteria=o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=max_iterations)
    )

    # 获取配准后的变换矩阵
    transformation_normalized = reg_icp.transformation

    # 反归一化配准结果
    transformation_denormalized = denormalize_transformation(transformation_normalized, scale_source, min_coords_source)

    # 应用反归一化的变换到源点云
    source_pcd_aligned = source.transform(transformation_denormalized)

    return source_pcd_aligned, transformation_denormalized


# 加载点云文件
input_file_source = r'/suyixuan/ABB/Pose_Estimation/Task6_Point_Cloud_Segment_and_Analyze/Dataset_Point_Cloud/normalized_realsense_point_cloud.ply'
input_file_target = r'/suyixuan/Open3D_ICP_Edition/stl_to_point_cloud_voxelization1_estimate_normals.ply'

source = o3d.io.read_point_cloud(input_file_source)
target = o3d.io.read_point_cloud(input_file_target)

# 可视化原始源点云
o3d.visualization.draw_geometries([source], window_name="Original Source Point Clouds")

# 可视化原始目标点云
o3d.visualization.draw_geometries([target], window_name="Original Target Point Clouds")




# 使用NICP进行配准
max_correspondence_distance = 0.5
max_iterations = 3000
aligned_source, transformation = apply_nicp(source, target, max_correspondence_distance, max_iterations)

# 可视化配准后的点云
o3d.visualization.draw_geometries([aligned_source, target], window_name="Aligned Point Clouds")

# 打印最终变换矩阵
print("Final Transformation Matrix:")
print(transformation)

# 保存配准后的点云
output_file = r'../Datasets/path_to_output_aligned_point_cloud.ply'
o3d.io.write_point_cloud(output_file, aligned_source)

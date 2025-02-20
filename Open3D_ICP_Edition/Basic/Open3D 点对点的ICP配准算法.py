"""
Author: Yixuan Su
Date: 2025/02/11 15:43
File: Open3D_Basic_Edition 点对点的ICP配准算法.py
Description: 

"""
import copy
import open3d as o3d
import numpy as np
# --------------------读取点云数据-------------------
source = o3d.io.read_point_cloud("../../../data/1.pcd")
target = o3d.io.read_point_cloud("../../../data/2.pcd")

# source = o3d.io.read_point_cloud(
#     r'E:\ABB-Project\ABB_wrs\suyixuan\ABB\Pose_Estimation\Task6_Point_Cloud_Segment_and_Analyze\Dataset_Point_Cloud\normalized_realsense_point_cloud.ply')
# target = o3d.io.read_point_cloud(
#     r'E:\ABB-Project\ABB_wrs\suyixuan\ABB\Pose_Estimation\Task9_Normalization_point_cloud\normalize_point_cloud_uniform_scaling\stl_to_point_cloud_voxelization1_estimate_normals_normalized_uniform.ply')

# ------------------可视化点云初始位置---------------
o3d.visualization.draw_geometries([source, target], width=600, height=600)
threshold = 1  # 距离阈值
trans_init = np.asarray([[1.0, 0.0, 0.0, 0.0],
                         [0.0, 1.0, 0.0, 0.0],
                         [0.0, 0.0, 1.0, 0],
                         [0.0, 0.0, 0.0, 1.0]])  # 初始变换矩阵，一般由粗配准提供
# -------------------------------------------------
# 计算两个重要指标，fitness计算重叠区域（内点对应关系/目标点数）。越高越好。
# inlier_rmse计算所有内在对应关系的均方根误差RMSE。越低越好。
print("Initial alignment")
evaluation = o3d.pipelines.registration.evaluate_registration(source, target, threshold, trans_init)
print(evaluation)  # 这里输出的是初始位置的 fitness和RMSE
print("Apply point-to-point Open3D_ICP_Edition")
icp_p2p = o3d.pipelines.registration.registration_icp(
        source, target, threshold, trans_init,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=35))     # 设置最大迭代次数
print(icp_p2p)  # 输出ICP相关信息
print("Transformation is:")
print(icp_p2p.transformation)  # 输出变换矩阵
# -----------------可视化配准结果--------------------


def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)         # 由于函数transformand paint_uniform_color会更改点云，
    target_temp = copy.deepcopy(target)         # 因此调用copy.deepcoy进行复制并保护原始点云。
    source_temp.paint_uniform_color([1, 0, 0])  # 点云着色
    target_temp.paint_uniform_color([0, 1, 0])
    source_temp.transform(transformation)
    o3d.io.write_point_cloud("../../../data/trans_of_source.pcd", source_temp)  # 保存点云
    o3d.visualization.draw_geometries([source_temp, target_temp], width=600, height=600)


draw_registration_result(source, target, icp_p2p.transformation)


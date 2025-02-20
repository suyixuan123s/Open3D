"""
Author: Yixuan Su
Date: 2025/02/16 16:47
File: Random-ICP实现点云精配准.py
Description: 

"""
import copy
import open3d as o3d
import numpy as np

# ---------------------------------读取点云数据-------------------------------------
source = o3d.io.read_point_cloud("../../../data/1.pcd")
target = o3d.io.read_point_cloud("../../../data/2.pcd")

# -------------------------------可视化点云初始位置----------------------------------
source.paint_uniform_color([1, 0, 0])  # 点云着色
target.paint_uniform_color([0, 1, 0])

o3d.visualization.draw_geometries([source, target], width=600, height=600)
threshold = 0.05  # 距离阈值
trans_init = np.asarray([[1.0, 0.0, 0.0, 0.0],
                         [0.0, 1.0, 0.0, 0.0],
                         [0.0, 0.0, 1.0, 0],
                         [0.0, 0.0, 0.0, 1.0]])  # 初始变换矩阵，一般由粗配准提供
print("\n")
print("Apply point-to-point Open3D_ICP_Edition")

# --------------------------Random-ICP实现点云精配准---------------------------------
source_random = source.random_down_sample(sampling_ratio=0.3)
icp_p2p = o3d.pipelines.registration.registration_icp(
    source_random, target, threshold, trans_init,
    o3d.pipelines.registration.TransformationEstimationPointToPoint(),
    o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=35))  # 设置最大迭代次数

print("\n")
print(icp_p2p)  # 输出ICP相关信息
print("Transformation is:")

print(icp_p2p.transformation)  # 输出变换矩阵
# --------------------------------结果可视化--------------------------------------
source_temp = copy.deepcopy(source)  # 由于函数transformand paint_uniform_color会更改点云，
target_temp = copy.deepcopy(target)  # 因此调用copy.deepcoy进行复制并保护原始点云。
source_temp.paint_uniform_color([1, 0, 0])  # 点云着色
target_temp.paint_uniform_color([0, 1, 0])
source_temp.transform(icp_p2p.transformation)
o3d.io.write_point_cloud("../../Open3D_Advanced_Edition/Open3D_Clustering/data/trans_of_source.pcd", source_temp)  # 保存点云
o3d.visualization.draw_geometries([source_temp, target_temp], width=600, height=600)

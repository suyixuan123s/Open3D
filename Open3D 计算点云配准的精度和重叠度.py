"""
Author: Yixuan Su
Date: 2025/02/18 21:46
File: Open3D 计算点云配准的精度和重叠度.py
Description: 

"""
import open3d as o3d
import numpy as np

# --------------------读取点云数据-------------------
source = o3d.io.read_point_cloud("../data/1.pcd")
target = o3d.io.read_point_cloud("../data/2.pcd")
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


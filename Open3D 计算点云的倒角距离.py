"""
Author: Yixuan Su
Date: 2025/02/18 20:03
File: Open3D 计算点云的倒角距离.py
Description: 

"""

import open3d as o3d
import numpy as np

# --------------------读取点云数据-------------------
source = o3d.io.read_point_cloud("../data/1.pcd")
target = o3d.io.read_point_cloud("../data/2.pcd")
# ---------------------距离计算---------------------
dists_a = source.compute_point_cloud_distance(target)
sum_a = np.sum(dists_a)
dists_b = target.compute_point_cloud_distance(source)
sum_b = np.sum(dists_b)
# --------------------计算倒角距离-------------------
s = len(source.points)  # 源点云中点的个数
t = len(target.points)  # 目标点云中点的个数
cham = sum_a / s + sum_b / t
print("倒角距离为:", cham)

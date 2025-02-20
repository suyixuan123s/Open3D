"""
Author: Yixuan Su
Date: 2025/02/19 20:52
File: 点云半径滤波.py
Description: 

"""
import laspy
import numpy as np
from scipy.spatial import cKDTree

las = laspy.read("tree.las")
dataset = np.vstack((las.x, las.y, las.z)).transpose()

# 使用scipy库中的kd树函数建立kd-tree索引
tree = cKDTree(dataset)
# ----------------------半径滤波-----------------------------
dis = 0.1  # 搜索半径
k = 10  # 邻居点数量
idxs = []  # 记录滤波后的点云idxs
for i in range(len(las.x)):
    neighbors_indices = tree.query_ball_point(dataset[i], dis)
    num = len(neighbors_indices)
    if num >= k:
        idxs.append(i)
# --------------保存滤波后的点云las文件-----------------------
ground_las = laspy.LasData(las.header)
ground_las.points = las[idxs].copy()
ground_las.write("ror_filtered_tree.las")

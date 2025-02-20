"""
Author: Yixuan Su
Date: 2025/02/19 22:14
File: 点云统计滤波.py
Description: 

"""
import laspy
from scipy.spatial import cKDTree
import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d


def las_viewer(las_cloud):
    xyz = np.vstack((las_cloud.x, las_cloud.y, las_cloud.z)).transpose()
    color = np.vstack((las_cloud.red / 65025, las_cloud.green / 65025, las_cloud.blue / 65025)).transpose()
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz)
    pcd.colors = o3d.utility.Vector3dVector(color)
    o3d.visualization.draw_geometries([pcd])


las = laspy.read("TLS SampleData.las")
las_viewer(las)
dataset = np.vstack((las.x, las.y, las.z)).transpose()
# 使用scipy库中的kd树函数建立kd-tree索引
tree = cKDTree(dataset)
k_dist = np.zeros_like(las.x)
# ----------------------统计滤波-----------------------------
k = 50  # 近邻点数
sigma = 1  # 标准差倍数
for i in range(len(las.x)):
    neighbors_distance, neighbors_indices = tree.query(dataset[i], k)
    k_dist[i] = np.sum(neighbors_distance)
max_distance = np.mean(k_dist) + sigma * np.std(k_dist)
# --------------保存滤波后的点云las文件-----------------------
ground_las = laspy.LasData(las.header)
ground_las.points = las[k_dist < max_distance].copy()
ground_las.write("filtered_tree.las")
las_viewer(ground_las)
# --------------------可视化点云-----------------------------
# x = las.X[k_dist < max_distance]
# y = las.Y[k_dist < max_distance]
# z = las.Z[k_dist < max_distance]
# fig = plt.figure(figsize=(12, 10), dpi=80)
# ax = fig.add_subplot(111, projection='3d')
# ax.scatter(x, y, z, s=3, edgecolor="red", marker=".")
# plt.show()


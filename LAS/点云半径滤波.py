"""
Author: Yixuan Su
Date: 2025/02/19 22:11
File: 点云半径滤波.py
Description: 

"""
import laspy
import numpy as np
from scipy.spatial import cKDTree
import open3d as o3d

las = laspy.read("TLS SampleData.las")
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

def las_viewer(las_cloud):
    xyz = np.vstack((las_cloud.x, las_cloud.y, las_cloud.z)).transpose()
    color = np.vstack((las_cloud.red / 65025, las_cloud.green / 65025, las_cloud.blue / 65025)).transpose()
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz)
    pcd.colors = o3d.utility.Vector3dVector(color)
    o3d.visualization.draw_geometries([pcd])

ground_las = laspy.LasData(las.header)
ground_las.points = las[idxs].copy()
ground_las.write("ror_filtered_tree.las")

las = laspy.read("TLS SampleData.las")
las_viewer(las)







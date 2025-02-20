"""
Author: Yixuan Su
Date: 2025/02/18 21:11
File: Open3D 基于投影点密度的建筑物立面提取.py
Description: 

"""

import open3d as o3d
import numpy as np

# --------------------读取点云-------------------------
cloud = o3d.io.read_point_cloud("../data/touyingdianceshi.pcd")
o3d.visualization.draw_geometries([cloud], window_name="原始点云")
step = 0.5  # 格网大小
th = 200  # 密度阈值
# --------------------创建格网-------------------------
point_cloud = np.asarray(cloud.points)
# 1、获取点云数据边界
x_min, y_min, z_min = np.amin(point_cloud, axis=0)
x_max, y_max, z_max = np.amax(point_cloud, axis=0)
# 2、计算格网行列数
width = np.ceil((x_max - x_min) / step)
height = np.ceil((y_max - y_min) / step)
print("像素格网的大小为： {} x {}".format(width, height))
# 初始化密度矩阵
M = np.ones((int(width), int(height)))
# 密度统计
for i in range(len(point_cloud)):
    row = np.floor((point_cloud[i][0] - x_min) / step)
    col = np.floor((point_cloud[i][1] - y_min) / step)
    M[int(row), int(col)] += 1

ind = list()
for i in range(len(point_cloud)):
    row = np.floor((point_cloud[i][0] - x_min) / step)
    col = np.floor((point_cloud[i][1] - y_min) / step)
    if M[int(row), int(col)] > th:
        ind.append(i)
density_cloud = cloud.select_by_index(ind)
o3d.io.write_point_cloud("立面点云.pcd", cloud)
o3d.visualization.draw_geometries([density_cloud], window_name="DensityPointCloud")

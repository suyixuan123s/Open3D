"""
Author: Yixuan Su
Date: 2025/02/18 20:24
File: Open3D 格网法计算点云的占地面积.py
Description: 

"""

import open3d as o3d
import numpy as np

# --------------------读取点云-------------------------
cloud = o3d.io.read_point_cloud("../data/CSite1_orig-utm.pcd")
o3d.visualization.draw_geometries([cloud], window_name="机载点云")

step = 2.0  # 格网大小

# --------------------创建格网-------------------------
point_cloud = np.asarray(cloud.points)
# 1、获取点云数据边界
x_min, y_min, z_min = np.amin(point_cloud, axis=0)
x_max, y_max, z_max = np.amax(point_cloud, axis=0)
# 2、计算格网行列数
width = int(np.ceil((x_max - x_min) / step))
height = int(np.ceil((y_max - y_min) / step))
print("像素格网的大小为： {} x {}".format(width, height))
# 3、初始化统计矩阵
M = np.zeros((int(width), int(height)))
# 4、格网点数统计
for i in range(len(point_cloud)):
    row = np.floor((point_cloud[i][0] - x_min) / step)
    col = np.floor((point_cloud[i][1] - y_min) / step)
    M[int(row), int(col)] += 1
# 5、如果格网内有点，则计算面积
ind = 0
for i in range(width):
    for j in range(height):
        if M[i, j] > 0:
            ind = ind + 1

grid_area = ind * step * step

print("格网法计算的占地面积为:", grid_area)

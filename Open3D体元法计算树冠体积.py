"""
Author: Yixuan Su
Date: 2025/02/18 10:25
File: Open3D体元法计算树冠体积.py
Description: 

"""
import open3d as o3d
import numpy as np

# --------------------------------------------读取点云-------------------------------------------
cloud = o3d.io.read_point_cloud("../data/bgm_cluster1.pcd")
o3d.visualization.draw_geometries([cloud], window_name="原始点云")
step = 0.05  # 格网大小
# -----------------------------------------获取点云坐标及范围-------------------------------------
point_cloud = np.asarray(cloud.points)

x_min, y_min, z_min = np.amin(point_cloud, axis=0)
x_max, y_max, z_max = np.amax(point_cloud, axis=0)
# ------------------------------------------计算总的行列层号--------------------------------------
row = np.ceil((x_max - x_min) / step)
col = np.ceil((y_max - y_min) / step)
cel = np.ceil((z_max - z_min) / step)
print("体素格网的大小为： {} x {} x {}".format(row, col, cel))
# 初始化统计矩阵
M = np.zeros((int(row), int(col), int(cel)))
# 点数统计
for i in range(len(point_cloud)):
    rID = np.floor((point_cloud[i][0] - x_min) / step)
    cID = np.floor((point_cloud[i][1] - y_min) / step)
    eID = np.floor((point_cloud[i][2] - z_min) / step)
    M[int(rID), int(cID), int(eID)] += 1
num = 0
for i in range(int(row)):
    for j in range(int(col)):
        for k in range(int(cel)):
            if M[int(i), int(j), int(k)] > 0:
                num += 1

# -------------------------------------------计算结果--------------------------------------------
gridVolume = num * np.power(step, 3)
print("体积大小为： {} 立方米".format(gridVolume))

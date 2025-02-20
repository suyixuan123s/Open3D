"""
Author: Yixuan Su
Date: 2025/02/18 20:22
File: Open3D 等间距抽稀算法.py
Description: 

"""
import numpy as np
import open3d as o3d

# ------------------------------读取点云---------------------------------------
pcd = o3d.io.read_point_cloud("../data/JZDM.pcd")
o3d.visualization.draw_geometries([pcd])
A = np.asarray(pcd.points)  # 点坐标
# -----------------------------等间距抽稀--------------------------------------
step = 10  # 采样间隔
firstIdx = np.random.randint(0, step + 1, size=1)  # 随机选取第1个采样点

sampleA = []
for i in range(firstIdx[0], A.shape[0], step):
    sampleA.append(i)
sampled_cloud = pcd.select_by_index(sampleA)
print('原始点云', pcd)
print('随机采样后的点云', sampled_cloud)
o3d.visualization.draw_geometries([sampled_cloud])

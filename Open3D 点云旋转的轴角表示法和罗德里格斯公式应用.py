"""
Author: Yixuan Su
Date: 2025/02/18 11:30
File: Open3D 点云旋转的轴角表示法和罗德里格斯公式应用.py
Description: 

"""
import open3d as o3d
import numpy as np
import copy

# -------------------------------读取点云--------------------------------------
pcd = o3d.io.read_point_cloud("../data/bunny.pcd")
pcd.paint_uniform_color([0, 0, 1])
# --------------------------------参数-----------------------------------------
N = np.array([0, 0, 1])  # 旋转轴，可任意设置
rad = np.pi / 4  # 旋转角度
# -----------------------------轴角转旋转矩阵-----------------------------------
N = N / np.linalg.norm(N)
A, B, C = N[0], N[1], N[2]
T = np.eye(4)
T[0, 0] = A * A * (1 - np.cos(rad)) + np.cos(rad)
T[0, 1] = A * B * (1 - np.cos(rad)) - C * np.sin(rad)
T[0, 2] = A * C * (1 - np.cos(rad)) + B * np.sin(rad)
T[1, 0] = A * B * (1 - np.cos(rad)) + C * np.sin(rad)
T[1, 1] = B * B * (1 - np.cos(rad)) + np.cos(rad)
T[1, 2] = B * C * (1 - np.cos(rad)) - A * np.sin(rad)
T[2, 0] = A * C * (1 - np.cos(rad)) - B * np.sin(rad)
T[2, 1] = B * C * (1 - np.cos(rad)) + A * np.sin(rad)
T[2, 2] = C * C * (1 - np.cos(rad)) + np.cos(rad)

print('旋转矩阵为：\n', T[:3, :3])
# ----------------------------点云变换-----------------------------------------
pcd_t = copy.deepcopy(pcd).transform(T)
pcd_t.paint_uniform_color([0, 1, 0])
o3d.io.write_point_cloud("bunny_AngleAixs.pcd", pcd_t)
o3d.visualization.draw_geometries([pcd, pcd_t])

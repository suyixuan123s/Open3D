"""
Author: Yixuan Su
Date: 2025/02/18 22:10
File: Open3D 点云镜像变换.py
Description: 

"""
import open3d as o3d
import numpy as np
import copy

# -------------------------------读取点云--------------------------------------
mesh = o3d.io.read_point_cloud("../data/bunny.ply")
mesh.paint_uniform_color([0, 0, 1])
# --------------------------------参数-----------------------------------------
N = np.array([0, 1, 0])  # 镜像平面
# -----------------------------计算镜像矩阵-------------------------------------
N = N / np.linalg.norm(N)
A, B, C = N[0], N[1], N[2]
T = np.eye(4)
T[0, 0] = 1 - 2 * A * A;
T[0, 1] = -2 * A * B;
T[0, 2] = -2 * A * C
T[1, 0] = -2 * A * B;
T[1, 1] = 1 - 2 * B * B;
T[1, 2] = -2 * B * C
T[2, 0] = -2 * A * C;
T[2, 1] = -2 * B * C;
T[2, 2] = 1 - 2 * C * C

print('镜像矩阵为：\n', T[:3, :3])
# ----------------------------点云变换-----------------------------------------
mesh_t = copy.deepcopy(mesh).transform(T)
mesh_t.paint_uniform_color([0, 1, 0])
o3d.io.write_point_cloud("bunny_image.pcd", mesh_t)
o3d.visualization.draw_geometries([mesh, mesh_t])

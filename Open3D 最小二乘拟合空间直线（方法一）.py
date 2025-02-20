"""
Author: Yixuan Su
Date: 2025/02/18 11:24
File: Open3D 最小二乘拟合空间直线（方法一）.py
Description: 

"""
import open3d as o3d
import numpy as np

# -------------------------------------读取点云--------------------------------------
pcd = o3d.io.read_point_cloud("../data/bunny.pcd")
# ---------------------------------最小二乘拟合空间直线-------------------------------
center = pcd.get_center()  # 计算点云质心
points = np.asarray(pcd.points)  # 获取点坐标
H = points - center  # 去质心
U, S, V = np.linalg.svd(H)  # 矩阵奇异值分解

# 点云最大特征值对应的特征向量即为拟合直线的方向向量A = (l,m,n)
A = V[0, :]
# 输出拟合结果
print('直线的方向向量为：', A)
print('直线上一点坐标为：', center)

# ----------------------------------结果可视化----------------------------------------
# 计算点云最值,用以确定可视化拟合直线的长度
max_pts = pcd.get_max_bound()
min_pts = pcd.get_min_bound()
line_length = np.linalg.norm(max_pts - min_pts)
mesh_cylinder = o3d.geometry.TriangleMesh.create_cylinder(radius=0.01, height=line_length)
mesh_cylinder.compute_vertex_normals()
mesh_cylinder.paint_uniform_color([0, 1, 0])
# 计算旋转矩阵用于可视化直线的方向
w = np.cross([0, 0, 1], A)
c = np.dot([0, 0, 1], A)
s = np.linalg.norm(w)
Sx = np.asarray([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])
R = np.eye(3) + Sx + Sx.dot(Sx) * ((1 - c) / (s ** 2))
# 绘制可视化直线
mesh_cylinder = mesh_cylinder.rotate(R, center=[0, 0, 0])
mesh_cylinder = mesh_cylinder.translate((center[0], center[1], center[2]))
o3d.visualization.draw_geometries([pcd, mesh_cylinder],
                                  window_name="最小二乘拟合空间直线",
                                  width=1024, height=768)

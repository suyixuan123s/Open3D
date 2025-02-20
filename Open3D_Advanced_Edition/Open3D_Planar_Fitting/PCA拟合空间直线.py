"""
Author: Yixuan Su
Date: 2025/02/16 17:55
File: PCA拟合空间直线.py
Description: 

"""

import numpy as np
import open3d as o3d
from sklearn.decomposition import PCA

# -----------------------------读取点云-----------------------------------
pcd = o3d.io.read_point_cloud("../../../data/bunny.pcd")
X = np.asarray(pcd.points)
# -----------------------------调用PCA------------------------------------
pca = PCA(n_components=3)  # 设置保留主成分个数
pca.fit(X)
eigenVector = pca.components_  # 按行排列，第一主成分排在首行
centrid = pca.mean_
# 点云最大特征值对应的特征向量即为拟合直线的方向向量A = (l,m,n)
A = eigenVector[0, :]
# 输出拟合结果
print('直线的方向向量为：', A)
print('直线上一点坐标为：', centrid)

# ----------------------------结果可视化------------------------------------
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
mesh_cylinder = mesh_cylinder.translate((centrid[0], centrid[1], centrid[2]))
o3d.visualization.draw_geometries([pcd, mesh_cylinder],
                                  window_name="PCA拟合空间直线",
                                  width=1024, height=768)

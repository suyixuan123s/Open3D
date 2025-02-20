"""
Author: Yixuan Su
Date: 2025/02/19 23:09
File: Open3D  RANSAC拟合空间圆.py
Description: 

"""

import open3d as o3d
import numpy as np
import pyransac3d as pyrsc

pcd_load = o3d.io.read_point_cloud("../data/two_tree.pcd")
points = np.asarray(pcd_load.points)
# --------------RANSAC拟合空间圆----------------------------
cir = pyrsc.Circle()
# center：圆心坐标，normal：圆所在平面的法向量，radius：圆的半径
# points：输入的三维坐标，thresh：内点的距离阈值，maxIteration：最大迭代次数
center, normal, radius, inliers = cir.fit(points, thresh=0.5, maxIteration=100)
print("center: " + str(center))
print("radius: " + str(radius))
print("vecC: " + str(normal))

# 根据向量获取转换矩阵，主要是为了可视化拟合出来的圆模型的。（无视即可）
R = pyrsc.get_rotationMatrix_from_vectors([0, 0, 1], normal)
# 获取位于拟合圆范围内的点云
inline = pcd_load.select_by_index(inliers).paint_uniform_color([1, 0, 0])

mesh_circle = o3d.geometry.TriangleMesh.create_torus(torus_radius=radius, tube_radius=0.1)
mesh_circle.compute_vertex_normals()
mesh_circle.paint_uniform_color([0.9, 0.1, 0.1])
mesh_circle = mesh_circle.rotate(R, center=[0, 0, 0])
mesh_circle = mesh_circle.translate((center[0], center[1], center[2]))
o3d.visualization.draw_geometries([mesh_circle, pcd_load, inline])


"""
Author: Yixuan Su
Date: 2025/02/19 23:07
File: Open3D  RANSAC三维点云球面拟合.py
Description: 

"""

import open3d as o3d
import numpy as np
import pyransac3d as pyrsc

pcd_load = o3d.io.read_point_cloud("../data/bgm_cluster1.pcd")
o3d.visualization.draw_geometries([pcd_load])

points = np.asarray(pcd_load.points)
# --------------------------RANSAC拟合球----------------------
sph = pyrsc.Sphere()
# center：球心坐标，radius：球的半径
# points：输入的三维坐标，thresh：内点的距离阈值，maxIteration：最大迭代次数
center, radius, inliers = sph.fit(points, thresh=0.4, maxIteration=100)
print("center: " + str(center))
print("radius: " + str(radius))
# 获取内点和外点
inline = pcd_load.select_by_index(inliers).paint_uniform_color([1, 0, 0])
outline = pcd_load.select_by_index(inliers, invert=True).paint_uniform_color([0, 1, 0])
# 可视化拟合结果
mesh_circle = o3d.geometry.TriangleMesh.create_sphere(radius=radius)
mesh_circle.compute_vertex_normals()
mesh_circle.paint_uniform_color([0.9, 0.1, 0.1])
mesh_circle = mesh_circle.translate((center[0], center[1], center[2]))
o3d.visualization.draw_geometries([outline, mesh_circle, inline])


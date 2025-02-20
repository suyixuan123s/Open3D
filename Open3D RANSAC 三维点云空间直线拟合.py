"""
Author: Yixuan Su
Date: 2025/02/19 22:42
File: Open3D RANSAC 三维点云空间直线拟合.py
Description: 

"""
#------------------------------模拟数据-------------------------------------
# import open3d as o3d
# import numpy as np
# import pyransac3d as pyrsc
#
# print("生成圆柱模型并添加噪声")
# mesh_in = o3d.geometry.TriangleMesh.create_cylinder(radius=1, height=500.0)
# vertices = np.asarray(mesh_in.vertices)
# noise = 15
# vertices += np.random.logistic(0, noise, size=vertices.shape)
# mesh_in.vertices = o3d.utility.Vector3dVector(vertices)
# mesh_in.compute_vertex_normals()
# mesh_in.paint_uniform_color([0.2, 0.2, 0.8])
# o3d.visualization.draw_geometries([mesh_in])
# print('从带有噪声点的圆柱模型中采样2000个点作为待输入数据')
# pcd_load = mesh_in.sample_points_uniformly(number_of_points=2000)
# o3d.visualization.draw_geometries([pcd_load])
#
# print('获取待输入数据的三维坐标')
# points = np.asarray(pcd_load.points)
# # ---------------------RANSAC拟合直线-------------------------------
# print('进行RANSAC拟合直线，拟合结果为：')
# line = pyrsc.Line()
# # A:直线的斜率，B：直线的截距，inliers：内点索引，
# # thresh：内点的距离阈值
# # maxIteration：RANSAC算法的拟合次数
# A, B, inliers = line.fit(points, thresh=15, maxIteration=100)
# print('直线的斜率为：', A)
# print('直线的截距为：', B)
# R = pyrsc.get_rotationMatrix_from_vectors([0, 0, 1], A)
# plane = pcd_load.select_by_index(inliers).paint_uniform_color([1, 0, 0])
#
# mesh_cylinder = o3d.geometry.TriangleMesh.create_cylinder(radius=1, height=1000)
# mesh_cylinder.compute_vertex_normals()
# mesh_cylinder.paint_uniform_color([1, 0, 0])
# mesh_cylinder = mesh_cylinder.rotate(R, center=[0, 0, 0])
# mesh_cylinder = mesh_cylinder.translate((B[0], B[1], B[2]))
# o3d.visualization.draw_geometries([pcd_load, plane, mesh_cylinder])
#


import open3d as o3d
import numpy as np
import pyransac3d as pyrsc

pcd_load = o3d.io.read_point_cloud("Line.pcd")
pcd_load.paint_uniform_color([0, 0, 1])
print('获取输入数据的三维坐标')
points = np.asarray(pcd_load.points)
# ---------------------RANSAC拟合直线-------------------------------
print('进行RANSAC拟合直线，拟合结果为：')
line = pyrsc.Line()
# A:直线的斜率，B：直线的截距，inliers：内点索引，
# thresh：内点的距离阈值
# maxIteration：RANSAC算法的拟合次数
A, B, inliers = line.fit(points, thresh=0.05, maxIteration=500)
print('直线的三维斜率为：', A)
print('直线的截距为：', B)
R = pyrsc.get_rotationMatrix_from_vectors([0, 0, 1], A)
ransac_line = pcd_load.select_by_index(inliers)

mesh_cylinder = o3d.geometry.TriangleMesh.create_cylinder(radius=0.05, height=100)
mesh_cylinder.compute_vertex_normals()
mesh_cylinder.paint_uniform_color([1, 0, 0])
mesh_cylinder = mesh_cylinder.rotate(R, center=[0, 0, 0])
mesh_cylinder = mesh_cylinder.translate((B[0], B[1], B[2]))
o3d.visualization.draw_geometries([pcd_load, ransac_line, mesh_cylinder])


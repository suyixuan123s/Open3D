"""
Author: Yixuan Su
Date: 2025/02/19 23:10
File: Open3D  RANSAC三维点云平面拟合.py
Description: 

"""
import numpy as np
import open3d as o3d
import pyransac3d as pyrsc

# -------------------读取点云数据并可视化------------------------
pcd_load = o3d.io.read_point_cloud("../data/two_tree.pcd")
o3d.visualization.draw_geometries([pcd_load])
points = np.asarray(pcd_load.points)
# --------------------RANSAC平面拟合----------------------------
plano1 = pyrsc.Plane()
# best_eq：平面方程的系数A、B、C、D，best_inliers：内点，
# thresh距离阈值， maxIteration：迭代次数
best_eq, best_inliers = plano1.fit(points, thresh=0.01, maxIteration=100)
print('平面模型系数为：', best_eq)
# 获取位于最佳拟合平面上的点
plane = pcd_load.select_by_index(best_inliers).paint_uniform_color([1, 0, 0])
# 获取平面点云的包围框
obb = plane.get_oriented_bounding_box()
aabb = plane.get_axis_aligned_bounding_box()
obb.color = [0, 0, 1]
aabb.color = [0, 1, 0]
# 获取平面外的点云
not_plane = pcd_load.select_by_index(best_inliers, invert=True)

o3d.visualization.draw_geometries([not_plane, plane, obb, aabb])


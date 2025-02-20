"""
Author: Yixuan Su
Date: 2025/02/19 22:24
File: Open3D  点云构建Delaunay三角网.py
Description: 

"""
import open3d as o3d
import numpy as np
from scipy import spatial
import matplotlib.pyplot as plt

# 加载点云数据
pcd = o3d.io.read_point_cloud("../data/tree2.pcd")
# 可视化点云
o3d.visualization.draw_geometries([pcd])
# 获取点云三维坐标
points = np.asarray(pcd.points)
# 获取点云XY坐标
point2d = np.c_[points[:, 0], points[:, 1]]
# Delaunay三角化
tri = spatial.Delaunay(point2d)
# 可视化三角化结果
plt.figure()
ax = plt.subplot(aspect="equal")
spatial.delaunay_plot_2d(tri, ax=ax)
plt.title("Point cloud delaunay  triangulation")
plt.show()

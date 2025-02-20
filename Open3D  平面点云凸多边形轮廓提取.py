"""
Author: Yixuan Su
Date: 2025/02/19 22:25
File: Open3D  平面点云凸多边形轮廓提取.py
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
# 获取平面点云的凸多边形边界
ch2d = spatial.ConvexHull(point2d)
# 可视化凸多边形边界结果
'''
# 方法一：matplot可视化
poly = plt.Polygon(point2d[ch2d.vertices], fill=None, lw=2, color="r", alpha=0.5)
ax = plt.subplot(aspect="equal")
plt.plot(point2d[:, 0], point2d[:, 1], 'go')
for i, pos in enumerate(point2d):
    plt.text(pos[0], pos[1], str(i), color="blue")
ax.add_artist(poly)
plt.show()
'''
plt.figure()
# 方法二：直接可视化
ax = plt.subplot(aspect="equal")
spatial.convex_hull_plot_2d(ch2d, ax=ax)
plt.title("Point cloud convex hull")
plt.show()

"""
Author: Yixuan Su
Date: 2025/02/19 11:09
File: Open3D 获取指定高程的所有点.py
Description: 

"""

import numpy as np
import open3d as o3d


# -------------------读取点云数据并可视化------------------------
pcd = o3d.io.read_point_cloud("../data/Stanford University/Lucy/lucy.pcd")
o3d.visualization.draw_geometries([pcd], window_name="原始点云")
# --------------------获取指定高程的点---------------------------
points = np.asarray(pcd.points)
Hz = points[5005000, 2]  # 选取第5005000个点的高程作为阈值
precision = 0.01         # 设置点云坐标的精度
ind = np.where((points[:, 2] >= Hz - precision) & (points[:, 2] <= Hz + precision))[0]
z_cloud = pcd.select_by_index(ind)
o3d.io.write_point_cloud("指定高程的点.pcd", z_cloud)
o3d.visualization.draw_geometries([z_cloud], window_name="获取指定高程的点")


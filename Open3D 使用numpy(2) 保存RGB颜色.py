"""
Author: Yixuan Su
Date: 2025/02/19 20:36
File: Open3D 使用numpy(2) 保存RGB颜色.py
Description: 

"""
# #---------------------------生成自定义颜色---------------------------------
# import numpy as np
# import open3d as o3d
#
# # -------------------------读取点云--------------------------
# pcd = o3d.io.read_point_cloud("../data/bunny.pcd")
# points = np.asarray(pcd.points)  # 点坐标
# num = np.asarray(pcd.points).shape[0]  # 点的个数
# # -------------------------生成颜色---------------------------
# colors = np.zeros([num, 3])
# height_max = np.max(points[:, 2])
# height_min = np.min(points[:, 2])
# delta_c = abs(height_max - height_min) / (255 * 2)
# for j in range(points.shape[0]):
#     color_n = (points[j, 2] - height_min) / delta_c
#     if color_n <= 255:
#         colors[j, :] = [0, 1 - color_n / 255, 1]
#     else:
#         colors[j, :] = [(color_n - 255) / 255, 0, 1]
# # -----------------------将颜色保存至pcd------------------------
# pcd.colors = o3d.utility.Vector3dVector(colors)  # 将颜色存入pcd
# o3d.io.write_point_cloud("color_bunny.pcd", pcd)
# o3d.visualization.draw_geometries([pcd])


# #------------------单一颜色 （方法一）--------------------------------------------
#
# import numpy as np
# import open3d as o3d
#
# # -------------------------读取点云--------------------------
# pcd = o3d.io.read_point_cloud("../data/bunny.pcd")
# points = np.asarray(pcd.points)  # 点坐标
# num = np.asarray(pcd.points).shape[0]  # 点的个数
# # -------------------------生成颜色---------------------------
# colors = np.zeros([num, 3])
# for i in range(num):
#     colors[i, :] = [0, 1, 0]  # 绿色
# # -----------------------将颜色保存至pcd------------------------
# pcd.colors = o3d.utility.Vector3dVector(colors)  # 将颜色存入pcd
# o3d.io.write_point_cloud("color_bunny.pcd", pcd)
# o3d.visualization.draw_geometries([pcd])

#
# ------------------单一颜色 （方法二）--------------------------------------------
#
# import open3d as o3d
#
# # -------------------------读取点云--------------------------
# pcd = o3d.io.read_point_cloud("../data/bunny.pcd")
# # -----------------------将颜色保存至pcd------------------------
# pcd.paint_uniform_color([0, 1, 0])
# o3d.io.write_point_cloud("color_bunny.pcd", pcd)
# o3d.visualization.draw_geometries([pcd])
#



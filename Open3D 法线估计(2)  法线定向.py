"""
Author: Yixuan Su
Date: 2025/02/19 20:19
File: Open3D 法线估计(2)  法线定向.py
Description: 

"""


# # 自定义方向
# import open3d as o3d
# import numpy as np
#
# pcd = o3d.io.read_point_cloud("../data/bunny.pcd")
# pcd.paint_uniform_color([1.0, 0.0, 0.0])
# pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.01, max_nn=30))
# pcd.orient_normals_to_align_with_direction([0, 0, 1])  # 自定义法线朝向
# print(np.asarray(pcd.normals)[:10, :])  # 输出前10个点的法向量
# o3d.visualization.draw_geometries([pcd], point_show_normal=True, window_name="法线估计",
#                                   width=1024, height=768,
#                                   left=50, top=50,
#                                   mesh_show_back_face=False)  # 可视化点云和法线


#-----------------------------相机方向-----------------------------------------------------------
#
# import open3d as o3d
# import numpy as np
#
# pcd = o3d.io.read_point_cloud("../data/bunny.pcd")
# pcd.paint_uniform_color([1.0, 0.0, 0.0])
# pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.01, max_nn=30))
# pcd.orient_normals_towards_camera_location([0, 0, 0])  # 朝向相机位置
# print(np.asarray(pcd.normals)[:10, :])  # 输出前10个点的法向量
# o3d.visualization.draw_geometries([pcd], point_show_normal=True, window_name="朝向相机位置",
#                                   width=1024, height=768,
#                                   left=50, top=50,
#                                   mesh_show_back_face=False)  # 可视化点云和法线



#---------------------------------------最小生成树----------------------------------------------

import open3d as o3d
import numpy as np

pcd = o3d.io.read_point_cloud("data//bunny.pcd")
pcd.paint_uniform_color([1.0, 0.0, 0.0])
pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.01, max_nn=30))
pcd.orient_normals_consistent_tangent_plane(10)  # 最小生成树
print(np.asarray(pcd.normals)[:10, :])  # 输出前10个点的法向量
o3d.visualization.draw_geometries([pcd], point_show_normal=True, window_name="朝向相机位置",
                                  width=1024, height=768,
                                  left=50, top=50,
                                  mesh_show_back_face=False)  # 可视化点云和法线


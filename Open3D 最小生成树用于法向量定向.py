"""
Author: Yixuan Su
Date: 2025/02/18 16:09
File: Open3D 最小生成树用于法向量定向.py
Description: 

"""
import open3d as o3d
import numpy as np

pcd = o3d.io.read_point_cloud("../data//bunny.pcd")
pcd.paint_uniform_color([1.0, 0.0, 0.0])
pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.01, max_nn=30))
pcd.orient_normals_consistent_tangent_plane(10)  # 最小生成树
print(np.asarray(pcd.normals)[:10, :])  # 输出前10个点的法向量
o3d.visualization.draw_geometries([pcd], point_show_normal=True, window_name="朝向相机位置",
                                  width=1024, height=768,
                                  left=50, top=50,
                                  mesh_show_back_face=False)  # 可视化点云和法线


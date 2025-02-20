"""
Author: Yixuan Su
Date: 2025/02/11 21:37
File: Open3D_Basic_Edition 均匀下采样.py
Description: 

"""
import open3d as o3d
import numpy as np

pcd = o3d.io.read_point_cloud("data/bunny.pcd")
print("原始点云中点的个数为：", np.asarray(pcd.points).shape[0])
# o3d.visualization.draw_geometries([pcd])
print("每5个点来降采样一个点")
uni_down_pcd = pcd.uniform_down_sample(every_k_points=5)
print("下采样之后点的个数为：", np.asarray(uni_down_pcd.points).shape[0])
o3d.visualization.draw_geometries([uni_down_pcd],
                                  window_name="均匀下采样",
                                  width=1200, height=800,
                                  left=50, top=50)


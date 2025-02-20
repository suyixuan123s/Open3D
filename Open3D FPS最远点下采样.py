"""
Author: Yixuan Su
Date: 2025/02/18 11:03
File: Open3D FPS最远点下采样.py
Description: 

"""

import numpy as np
import open3d as o3d

pcd = o3d.io.read_point_cloud('../data/cloud_bin_0.pcd')
count = len(np.asarray(pcd.points))
if count == 0:  # 如果没读取到数据则退出程序
    exit()

o3d.visualization.draw_geometries([pcd], window_name="采样前的点云",
                                  width=1024, height=768,
                                  left=50, top=50,
                                  mesh_show_back_face=False)

fps_down_pcd = pcd.farthest_point_down_sample(5000)  # 采样点的个数
o3d.io.write_point_cloud("test1.pcd", fps_down_pcd)
o3d.visualization.draw_geometries([fps_down_pcd], window_name="采样后的点云",
                                  width=1024, height=768,
                                  left=50, top=50,
                                  mesh_show_back_face=False)


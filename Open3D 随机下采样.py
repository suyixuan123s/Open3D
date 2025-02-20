"""
Author: Yixuan Su
Date: 2025/02/19 20:40
File: Open3D 随机下采样.py
Description: 

"""
import open3d as o3d
import numpy as np

pcd = o3d.io.read_point_cloud("../data/bunny.pcd")
print("原始点云中点的个数为：", np.asarray(pcd.points).shape[0])
# o3d.visualization.draw_geometries([pcd])
print("下采样至原来的十分之一")
uni_down_pcd = pcd.random_down_sample(sampling_ratio=0.1)
print("下采样之后点的个数为：", np.asarray(uni_down_pcd.points).shape[0])
o3d.visualization.draw_geometries([uni_down_pcd],
                                  window_name="随机下采样",
                                  width=1200, height=800,
                                  left=50, top=50)


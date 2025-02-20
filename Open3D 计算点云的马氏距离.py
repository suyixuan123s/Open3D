"""
Author: Yixuan Su
Date: 2025/02/19 20:13
File: Open3D 计算点云的马氏距离.py
Description: 

"""
import open3d as o3d
import numpy as np
from matplotlib import pyplot as plt

# -------------------------读取点云-----------------------------
pcd = o3d.io.read_point_cloud("../data/Stanford University/horse/horse.pcd")
print(pcd)
# -----------------------计算马氏距离------------------------
madist = pcd.compute_mahalanobis_distance()
madist = np.array(madist)
print(madist)
# ----------------------使用伪颜色显示最近邻点-------------------
density_colors = plt.get_cmap('plasma')(
    (madist - madist.min()) / (madist.max() - madist.min()))
density_colors = density_colors[:, :3]
pcd.colors = o3d.utility.Vector3dVector(density_colors)
o3d.visualization.draw_geometries([pcd], window_name="计算马氏距离",
                                  width=1024, height=768,
                                  left=50, top=50,
                                  mesh_show_back_face=False)

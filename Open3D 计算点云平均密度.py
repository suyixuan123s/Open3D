"""
Author: Yixuan Su
Date: 2025/02/19 20:12
File: Open3D 计算点云平均密度.py
Description: 

"""
import open3d as o3d
import numpy as np
from matplotlib import pyplot as plt

# -------------------------读取点云-----------------------------
pcd = o3d.io.read_point_cloud("../data/tree2.pcd")
print(pcd)
# ------------------------计算平均密度--------------------------
nndist = pcd.compute_nearest_neighbor_distance()
nndist = np.array(nndist)
density = np.mean(nndist)  # 计算平均密度
print("点云密度为 denstity=", density)
# ---------------------使用伪颜色显示最近邻点--------------------
density_colors = plt.get_cmap('hot')(
    (nndist - nndist.min()) / (nndist.max() - nndist.min()))
density_colors = density_colors[:, :3]
pcd.colors = o3d.utility.Vector3dVector(density_colors)
o3d.visualization.draw_geometries([pcd], window_name="计算平均密度",
                                  width=1024, height=768,
                                  left=50, top=50,
                                  mesh_show_back_face=False)

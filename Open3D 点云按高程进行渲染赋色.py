"""
Author: Yixuan Su
Date: 2025/02/18 15:39
File: Open3D 点云按高程进行渲染赋色.py
Description: 

"""
import numpy as np
import open3d as o3d

# -----------------------------------读取点云-----------------------------------
pcd = o3d.io.read_point_cloud("../data/飞机1.pcd")
points = np.asarray(pcd.points)

# --------------------------根据高度生成颜色（详细过程）--------------------------
colors = np.zeros([points.shape[0], 3])
z_max = np.max(points[:, 2])
z_min = np.min(points[:, 2])
delta_z = abs(z_max - z_min) / (255 * 2)
for i in range(points.shape[0]):
    color_n = (points[i, 2] - z_min) / delta_z
    if color_n <= 255:
        colors[i, :] = [0, 1 - color_n / 255, 1]
    else:
        colors[i, :] = [(color_n - 255) / 255, 0, 1]
# o3d.io.write_point_cloud("render.pcd", pcd)
# --------------------------------可视化结果------------------------------------
pcd.colors = o3d.utility.Vector3dVector(colors)
o3d.visualization.draw_geometries([pcd], window_name="点云按高程进行赋色",
                                  width=1024, height=768,
                                  left=50, top=50,
                                  mesh_show_back_face=False)

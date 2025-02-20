"""
Author: Yixuan Su
Date: 2025/02/18 15:29
File: Open3D 基于点云高程制作热力图.py
Description: 

"""
import numpy as np
import open3d as o3d
from matplotlib import pyplot as plt

# -----------------------------------读取点云-----------------------------------
pcd = o3d.io.read_point_cloud("../data/飞机1.pcd")
points = np.asarray(pcd.points)
# ------------------------------用来计算Z值取值范围------------------------------
zdist = points[:, 2]
# ----------------------------使用伪颜色对点云进行渲染---------------------------
zhot_colors = plt.get_cmap('hot')(  # hot表示为热力图
    (zdist - zdist.min()) / (zdist.max() - zdist.min()))
zhot_colors = zhot_colors[:, :3]
pcd.colors = o3d.utility.Vector3dVector(zhot_colors)
o3d.io.write_point_cloud("render.pcd", pcd)
o3d.visualization.draw_geometries([pcd], window_name="点云热力图渲染赋色",
                                  width=1024, height=768,
                                  left=50, top=50,
                                  mesh_show_back_face=False)

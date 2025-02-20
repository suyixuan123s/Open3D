"""
Author: Yixuan Su
Date: 2025/02/19 19:58
File: Open3D 计算点云质心.py
Description: 

"""

import open3d as o3d

# -----------------------------读取点云--------------------------------
pcd = o3d.io.read_point_cloud("../data/ro.pcd")
# ----------------------------计算点云质心------------------------------
center = pcd.get_center()
print("点云质心为：", center)
# ----------------------------可视化点云-------------------------------
o3d.visualization.draw_geometries([pcd], window_name="显示点云",
                                  width=1024, height=768,
                                  left=50, top=50,
                                  mesh_show_back_face=False)


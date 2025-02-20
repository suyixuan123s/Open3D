"""
Author: Yixuan Su
Date: 2025/02/19 20:00
File: Open3D 计算点云坐标最值.py
Description: 

"""
import open3d as o3d

# -----------------------------读取点云--------------------------------
pcd = o3d.io.read_point_cloud("../data/bunny.pcd")
# ----------------------------计算点云最值------------------------------
max_pts = pcd.get_max_bound()
print("点云坐标最大值为：", max_pts)
min_pts = pcd.get_min_bound()
print("点云坐标最小值为：", min_pts)
# ----------------------------可视化点云-------------------------------
o3d.visualization.draw_geometries([pcd], window_name="显示点云",
                                  width=1024, height=768,
                                  left=50, top=50,
                                  mesh_show_back_face=False)


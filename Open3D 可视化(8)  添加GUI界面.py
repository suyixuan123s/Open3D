"""
Author: Yixuan Su
Date: 2025/02/19 20:17
File: Open3D 可视化(8)  添加GUI界面.py
Description: 

"""

# # --------------------------初始化应用窗口--------------------------
# app = gui.Application.instance
# app.initialize()
# app.add_window(vis)  # 添加GUI界面
# app.run()  # 启动GUI界面


import open3d as o3d
import open3d.visualization.gui as gui  # 导入gui模块

# --------------------------初始化应用窗口--------------------------
app = gui.Application.instance
app.initialize()
# ----------------------------读取点云-----------------------------
cloud = o3d.io.read_point_cloud("../data/bunny.pcd")
cloud.paint_uniform_color((0, 1, 0))
# ---------------------------计算点云凸包--------------------------
hull, _ = cloud.compute_convex_hull()
hull_ls = o3d.geometry.LineSet.create_from_triangle_mesh(hull)
hull_ls.paint_uniform_color((1, 0, 0))
# --------------------------计算AABB包围盒------------------------
aabb = cloud.get_axis_aligned_bounding_box()
aabb.color = (1, 0, 0)  # aabb包围盒为红色
# ----------------------------可视化-----------------------------
vis = o3d.visualization.O3DVisualizer("Open3D应用窗口", 1024, 768)
vis.show_settings = True
vis.add_geometry("Points", cloud)
vis.add_geometry("LineSet", hull_ls)
vis.add_geometry("AABB", aabb)
app.add_window(vis)  # 添加GUI界面
app.run()  # 启动GUI界面

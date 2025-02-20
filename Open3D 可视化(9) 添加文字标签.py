"""
Author: Yixuan Su
Date: 2025/02/19 10:28
File: Open3D 可视化(9) 添加文字标签.py
Description: 

"""
# import open3d as o3d
# import open3d.visualization.gui as gui
#
#
# # --------------------------初始化应用窗口--------------------------
# app = gui.Application.instance
# app.initialize()
# # ----------------------------读取点云-----------------------------
# cloud = o3d.io.read_point_cloud("../data/tree2.pcd")
#
# vis = o3d.visualization.O3DVisualizer("Open3D - Add 3D Text", 1024, 768)
# vis.show_settings = True
# vis.add_geometry("PointCloud", cloud)
# # ------------------每隔100个点添加一个标签，一共添加20个-------------
# for idx in range(0, 20):
#     vis.add_3d_label(cloud.points[idx+100], "{}".format(idx))
#
# vis.reset_camera_to_default()
#
# app.add_window(vis)
# app.run()


import numpy as np
import open3d as o3d
import open3d.visualization.gui as gui

# --------------------------初始化应用窗口--------------------------
app = gui.Application.instance
app.initialize()
# ----------------------------读取点云-----------------------------
cloud = o3d.io.read_point_cloud("../data/tree2.pcd")

vis = o3d.visualization.O3DVisualizer("Open3D - Add 3D Text", 1024, 768)
vis.show_settings = True
vis.add_geometry("PointCloud", cloud)
# ---------------------------计算点云质心--------------------------
center = cloud.get_center()
print("点云质心为：", center)
# -----------------------可视化质心并添加标签-----------------------
mesh_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.5,
                                                      resolution=100)
mesh_sphere.translate(center, relative=False)

mesh_sphere.compute_vertex_normals()
mesh_sphere.paint_uniform_color([1, 0, 0])
vis.add_geometry("CenterMesh", mesh_sphere)
vis.add_3d_label(center, "center")
vis.reset_camera_to_default()

app.add_window(vis)
app.run()


"""
Author: Yixuan Su
Date: 2025/02/19 20:23
File: Open3D 泊松盘采样.py
Description: 

"""
import open3d as o3d

# -----------------------------读取mesh模型----------------------------------
mesh = o3d.io.read_triangle_mesh("../data/Armadillo.ply")
mesh.paint_uniform_color([1, 0.7, 0])  # 给mesh渲染颜色
o3d.visualization.draw_geometries([mesh], window_name="Open3D显示mesh模型",
                                  width=1024, height=768,
                                  left=50, top=50,
                                  mesh_show_wireframe=True,  # 是否以格网线的形式显示
                                  mesh_show_back_face=False  # 是否显示面片背景
                                  )  # 显示mesh模型


# 使用init_factor X number_of_points 进行mesh上点云的采样
pcd = mesh.sample_points_poisson_disk(number_of_points=5000, init_factor=5)
o3d.visualization.draw_geometries([pcd], window_name="泊松盘在mesh上的采样",
                                  width=1024, height=768,
                                  left=50, top=50,
                                  mesh_show_back_face=False)


# ---------------------直接在点云上进行poisson_disk采样-----------------------
pcd = mesh.sample_points_poisson_disk(number_of_points=1000, pcl=pcd)
o3d.visualization.draw_geometries([pcd], window_name="泊松盘在点云上的采样",
                                  width=1024, height=768,
                                  left=50, top=50,
                                  mesh_show_back_face=False)

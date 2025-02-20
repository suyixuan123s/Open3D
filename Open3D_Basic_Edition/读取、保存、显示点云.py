"""
Author: Yixuan Su
Date: 2025/02/11 13:04
File: Open3D_Basic_Edition 读取、保存、显示点云【2025最新版】.py
Description: 

"""
import open3d as o3d

pcd = o3d.io.read_point_cloud("../../data/many_tree.pcd")

o3d.visualization.draw_geometries([pcd])


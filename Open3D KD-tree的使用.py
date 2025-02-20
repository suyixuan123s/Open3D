"""
Author: Yixuan Su
Date: 2025/02/19 22:38
File: Open3D KD-tree的使用.py
Description: 

"""
import open3d as o3d
import numpy as np

pcd = o3d.io.read_point_cloud("../data/1.pcd")
pcd.paint_uniform_color([0.5, 0.5, 0.5])  # 把所有点渲染为灰色（灰兔子）
pcd_tree = o3d.geometry.KDTreeFlann(pcd)  # 建立KD树索引
pcd.colors[200] = [1, 0, 0]  # 给定查询点并渲染为红色
# ---------------K近邻搜索----------------
[k, idx, _] = pcd_tree.search_knn_vector_3d(pcd.points[200], 200)  # K近邻搜索
np.asarray(pcd.colors)[idx[1:], :] = [0, 1, 0]  # K邻域的点，渲染为绿色
# ---------------半径搜索-----------------
pcd.colors[1500] = [1, 0, 0]  # 给定查询点并渲染为红色
[k1, idx1, _] = pcd_tree.search_radius_vector_3d(pcd.points[1500], 0.02)  # 半径搜索
np.asarray(pcd.colors)[idx1[1:], :] = [0, 0, 1]  # 半径搜索结果并渲染为蓝色
# ---------------混合搜索-----------------
pcd.colors[3000] = [1, 1, 0]  # 给定查询点并渲染为黄色
[k2, idx2, _] = pcd_tree.search_hybrid_vector_3d(pcd.points[3000], 0.05, 200)  # K近邻搜索
np.asarray(pcd.colors)[idx2[1:], :] = [0, 1, 0.8]  # 半径搜索结果并渲染为青色
o3d.visualization.draw_geometries([pcd])

"""
Author: Yixuan Su
Date: 2025/02/18 10:27
File: Open3D 模型切片.py
Description: 

"""

import open3d as o3d
import numpy as np

# ---------------------------------------读取mesh模型----------------------------------
file_name = '../data/bunny.ply'
# mesh = o3d.t.geometry.TriangleMesh.from_legacy(o3d.io.read_triangle_mesh(file_name))

mesh = o3d.t.io.read_triangle_mesh(file_name)
# -----------------------------------------mesh切片-----------------------------------
contours = mesh.slice_plane([0, 0, 0], [0, 1, 0], np.linspace(0, 0.2))
# -----------------------------------------结果可视化----------------------------------
o3d.visualization.draw([{'name': 'bunny', 'geometry': contours}])



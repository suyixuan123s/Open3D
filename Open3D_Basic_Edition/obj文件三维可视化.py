"""
Author: Yixuan Su
Date: 2025/02/11 13:19
File: Open3D_Basic_Edition obj文件三维可视化.py
Description:
"""

import open3d as o3d
import numpy as np

# obj面片显示
obj_mesh = o3d.io.read_triangle_mesh('pikaqiu.obj')
print(obj_mesh)
mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1, origin=[0, 0, 0])  # 添加坐标系
obj_mesh.compute_vertex_normals()
o3d.visualization.draw_geometries([obj_mesh, mesh_frame], window_name="Open3D1")


# obj顶点显示
pcobj = o3d.geometry.PointCloud()
pcobj.points = o3d.utility.Vector3dVector(obj_mesh.vertices)
o3d.visualization.draw_geometries([pcobj], window_name="Open3D2")
# obj顶点转array
obj_pc = np.asarray(obj_mesh.vertices)
print(obj_pc)

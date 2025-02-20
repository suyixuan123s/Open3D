"""
Author: Yixuan Su
Date: 2025/02/18 20:54
File: Open3D 模型锐化.py
Description: 

"""
import open3d as o3d

# ----------------------------------加载模型-----------------------------------
mesh = o3d.io.read_triangle_mesh("../data/Armadillo.ply")
# --------------------------------计算顶点法向量--------------------------------
mesh.compute_vertex_normals()
print(f'该模型有 {len(mesh.vertices)} 个顶点 和 {len(mesh.triangles)} 个三角形面片')
# -----------------------------------可视化------------------------------------
o3d.visualization.draw_geometries([mesh], mesh_show_wireframe=True)
# ----------------------------------模型锐化------------------------------------
mesh = mesh.filter_sharpen(number_of_iterations=2, strength=0.3)
# ---------------------------------结果可视化-----------------------------------
o3d.visualization.draw_geometries([mesh], mesh_show_wireframe=True)

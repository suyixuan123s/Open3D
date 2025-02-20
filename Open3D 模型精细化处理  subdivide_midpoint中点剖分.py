"""
Author: Yixuan Su
Date: 2025/02/18 20:59
File: Open3D 模型精细化处理  subdivide_midpoint中点剖分.py
Description: 

"""
import open3d as o3d

mesh = o3d.io.read_triangle_mesh("../data/Armadillo.ply")
mesh.compute_vertex_normals()
print(
    f'The mesh has {len(mesh.vertices)} vertices and {len(mesh.triangles)} triangles'
)
o3d.visualization.draw_geometries([mesh], mesh_show_wireframe=True)

# ----------------------------------中点剖分------------------------------------
mesh = mesh.subdivide_midpoint(number_of_iterations=2)
print(
    f'After subdivision it has {len(mesh.vertices)} vertices and {len(mesh.triangles)} triangles'
)
# ---------------------------------结果可视化-----------------------------------
o3d.visualization.draw_geometries([mesh], mesh_show_wireframe=True)

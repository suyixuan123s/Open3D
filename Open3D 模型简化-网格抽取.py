"""
Author: Yixuan Su
Date: 2025/02/18 20:45
File: Open3D 模型简化-网格抽取.py
Description: 

"""
import open3d as o3d

# -----------------------------加载模型--------------------------------
mesh = o3d.io.read_triangle_mesh("../data/bunny.ply")
# ---------------------------计算顶点法向量----------------------------
mesh.compute_vertex_normals()
print(f'原始模型中有 {len(mesh.vertices)} 个顶点和 {len(mesh.triangles)} 个三角面片。')
# -------------------------------可视化-------------------------------
o3d.visualization.draw_geometries([mesh])

voxel_size = max(mesh.get_max_bound() - mesh.get_min_bound()) / 32
print(f'格网边长为： = {voxel_size:e}')
# ----------------网格抽取 简化二次抽取 simplify_quadric_decimation--------
mesh_smp = mesh.simplify_quadric_decimation(
    target_number_of_triangles=6500)
print(f'简化之后的模型有 {len(mesh_smp.vertices)} 个顶点和 {len(mesh_smp.triangles)} 个三角面片。')
o3d.visualization.draw_geometries([mesh_smp])

mesh_smp = mesh.simplify_quadric_decimation(
    target_number_of_triangles=1700)
print(f'再次简化之后的模型有 {len(mesh_smp.vertices)} 个顶点和 {len(mesh_smp.triangles)} 个三角面片。')
o3d.visualization.draw_geometries([mesh_smp])

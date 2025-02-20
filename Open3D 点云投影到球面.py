"""
Author: Yixuan Su
Date: 2025/02/19 20:48
File: Open3D 点云投影到球面.py
Description: 

"""
import numpy as np
import open3d as o3d


def point_cloud_plane_project(cloud, coefficients):
    """
    点云投影到球面
    :param cloud:输入点云
    :param coefficients: 待投影的球面
    :return: 投影后的点云
    """
    # 获取球面系数
    xo = coefficients[0]
    yo = coefficients[1]
    zo = coefficients[2]
    r = coefficients[3]
    # 三维坐标执行投影
    points = np.asarray(cloud.points)
    xi = points[:, 0]
    yi = points[:, 1]
    zi = points[:, 2]
    # 投影到球面
    xp = xi * r / np.linalg.norm([xi, yi, zi], axis=0) + xo
    yp = yi * r / np.linalg.norm([xi, yi, zi], axis=0) + yo
    zp = zi * r / np.linalg.norm([xi, yi, zi], axis=0) + zo
    # 投影后的三维坐标
    project_points = np.c_[xp, yp, zp]
    project_cloud = o3d.geometry.PointCloud()  # 使用numpy生成点云
    project_cloud.points = o3d.utility.Vector3dVector(project_points)
    project_cloud.colors = pcd.colors  # 获取投影前对应的颜色赋值给投影后的点
    return project_cloud


# -------------------读取点云数据并可视化------------------------
pcd = o3d.io.read_point_cloud("../data/1.pcd")
# o3d.visualization.draw_geometries([pcd])
# ----------------------给定球面系数----------------------------
center = np.array([0.1181, -0.1363, -0.1567, 5.1674])
# -----------------------进行投影------------------------------
projected_cloud = point_cloud_plane_project(pcd, center)
o3d.io.write_point_cloud("project_cloud.pcd", projected_cloud)

# 可视化球面和投影后的点云
mesh_circle = o3d.geometry.TriangleMesh.create_sphere(radius=center[3])
mesh_circle.compute_vertex_normals()
mesh_circle.paint_uniform_color([0.9, 0.1, 0.1])
mesh_circle = mesh_circle.translate((center[0], center[1], center[2]))
o3d.visualization.draw_geometries([projected_cloud, mesh_circle], window_name="点云投影到球面",
                                  width=900, height=900,
                                  left=50, top=50,
                                  mesh_show_back_face=False)

"""
Author: Yixuan Su
Date: 2025/02/19 22:20
File: Open3D 点云投影到拟合平面.py
Description: 

"""
import numpy as np
import open3d as o3d


def point_cloud_plane_project(cloud, coefficients):
    """
    点云投影到平面
    :param cloud:输入点云
    :param coefficients: 待投影的平面
    :return: 投影后的点云
    """
    # 获取平面系数
    A = coefficients[0]
    B = coefficients[1]
    C = coefficients[2]
    D = coefficients[3]
    # 构建投影函数
    Xcoff = np.array([B * B + C * C, -A * B, -A * C])
    Ycoff = np.array([-B * A, A * A + C * C, -B * C])
    Zcoff = np.array([-A * C, -B * C, A * A + B * B])
    # 三维坐标执行投影
    points = np.asarray(cloud.points)
    xp = np.dot(points, Xcoff) - A * D
    yp = np.dot(points, Ycoff) - B * D
    zp = np.dot(points, Zcoff) - C * D
    project_points = np.c_[xp, yp, zp]  # 投影后的三维坐标
    project_cloud = o3d.geometry.PointCloud()  # 使用numpy生成点云
    project_cloud.points = o3d.utility.Vector3dVector(project_points)
    project_cloud.colors = pcd.colors  # 获取投影前对应的颜色赋值给投影后的点
    return project_cloud


# -------------------读取点云数据并可视化------------------------
pcd = o3d.io.read_point_cloud("../data/tree2.pcd")
# o3d.visualization.draw_geometries([pcd])
# --------------------RANSAC平面拟合----------------------------
plane_model, _ = pcd.segment_plane(distance_threshold=0.3,
                                   ransac_n=10,
                                   num_iterations=100)
projected_cloud = point_cloud_plane_project(pcd, plane_model)
# o3d.io.write_point_cloud("project_cloud.pcd", projected_cloud)
o3d.visualization.draw_geometries([projected_cloud], window_name="点云投影到平面",
                                  width=900, height=900,
                                  left=50, top=50,
                                  mesh_show_back_face=False)

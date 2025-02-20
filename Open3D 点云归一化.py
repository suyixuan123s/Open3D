"""
Author: Yixuan Su
Date: 2025/02/18 22:11
File: Open3D 点云归一化.py
Description: 

"""
import numpy as np
import open3d as o3d


def point_cloud_normalize(cloud):
    """
    对点云数据进行归一化
    :param cloud: 需要归一化的点云数据
    :return: 归一化后的点云数据
    """
    centroid = cloud.get_center()  # 计算点云质心
    points = np.asarray(cloud.points)
    points = points - centroid  # 去质心
    m = np.max(np.sqrt(np.sum(points ** 2, axis=1)))  # 计算点云中的点与坐标原点的最大距离
    points = points / m  # 对点云进行缩放
    normalize_cloud = o3d.geometry.PointCloud()  # 使用numpy生成点云
    normalize_cloud.points = o3d.utility.Vector3dVector(points)
    normalize_cloud.colors = cloud.colors  # 获取投影前对应的颜色赋值给投影后的点
    return normalize_cloud


# 反归一化：ret=normalized*m+center


# ----------------------读取点云数据-------------------------
pcd = o3d.io.read_point_cloud("../data/bunny.pcd")
# o3d.visualization.draw_geometries([pcd])
# -----------------------点云归一化--------------------------
normalized_cloud = point_cloud_normalize(pcd)
o3d.io.write_point_cloud("normalize_cloud.pcd", normalized_cloud)
o3d.visualization.draw_geometries([normalized_cloud], window_name="点云归一化",
                                  width=900, height=900,
                                  left=50, top=50,
                                  mesh_show_back_face=False)

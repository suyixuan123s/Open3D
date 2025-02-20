"""
Author: Yixuan Su
Date: 2025/02/19 22:22
File: Open3D 计算点云的表面曲率.py
Description: 

"""
import open3d as o3d
import numpy as np
from matplotlib import pyplot as plt


def pca_compute(data, sort=True):
    """
     SVD分解计算点云的特征值与特征向量
    :param data: 输入数据
    :param sort: 是否将特征值特征向量进行排序
    :return: 特征值与特征向量
    """
    average_data = np.mean(data, axis=0)  # 求均值
    decentration_matrix = data - average_data  # 去中心化
    H = np.dot(decentration_matrix.T, decentration_matrix)  # 求解协方差矩阵 H
    eigenvectors, eigenvalues, eigenvectors_T = np.linalg.svd(H)  # SVD求解特征值、特征向量

    if sort:
        sort = eigenvalues.argsort()[::-1]  # 降序排列
        eigenvalues = eigenvalues[sort]  # 索引

    return eigenvalues


def caculate_surface_curvature(cloud, radius=0.003):
    """
    计算点云的表面曲率
    :param cloud: 输入点云
    :param radius: k近邻搜索的半径，默认值为：0.003m
    :return: 点云中每个点的表面曲率
    """
    points = np.asarray(cloud.points)
    kdtree = o3d.geometry.KDTreeFlann(cloud)
    num_points = len(cloud.points)
    curvature = []  # 储存表面曲率
    for i in range(num_points):
        k, idx, _ = kdtree.search_radius_vector_3d(cloud.points[i], radius)

        neighbors = points[idx, :]
        w = pca_compute(neighbors)  # w为特征值
        delt = np.divide(w[2], np.sum(w), out=np.zeros_like(w[2]), where=np.sum(w) != 0)
        curvature.append(delt)
    curvature = np.array(curvature, dtype=np.float64)
    return curvature


if __name__ == '__main__':
    pcd = o3d.io.read_point_cloud("../data/1.pcd")
    surface_curvature = caculate_surface_curvature(pcd, radius=0.003)

    # ----------------------------使用伪颜色对点云进行渲染---------------------------
    curvature_colors = plt.get_cmap('hot')(  # hot表示为热力图
        (surface_curvature - surface_curvature.min()) / (surface_curvature.max() - surface_curvature.min()))
    curvature_colors = curvature_colors[:, :3]
    pcd.colors = o3d.utility.Vector3dVector(curvature_colors)
    # o3d.io.write_point_cloud("render.pcd", pcd)
    o3d.visualization.draw_geometries([pcd], window_name="点云曲率热力图渲染赋色",
                                      width=1024, height=768,
                                      left=50, top=50,
                                      mesh_show_back_face=False)

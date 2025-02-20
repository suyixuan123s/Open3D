"""
Author: Yixuan Su
Date: 2025/02/18 10:00
File: Open3D  利用点云的曲率密度提取特征点.py
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


def caculate_curvature_density(cloud, radius=0.003):
    """
    计算点云的曲率密度参数
    :param cloud: 输入点云
    :param radius: k近邻搜索的半径，默认值为：0.003m
    :return: 点云中每个点的曲率密度参数
    """
    points = np.asarray(cloud.points)
    kdtree = o3d.geometry.KDTreeFlann(cloud)
    num_points = len(cloud.points)
    curvature_density = []  # 储存曲率密度参数
    for i in range(num_points):
        k, idx, _ = kdtree.search_radius_vector_3d(cloud.points[i], radius)

        neighbors = points[idx, :]
        w = pca_compute(neighbors)  # w为特征值
        delt = np.divide(w[2], np.sum(w), out=np.zeros_like(w[2]), where=np.sum(w) != 0)
        curvature_density.append(delt * k)  # 计算曲率密度特征参数
    cdensity = np.array(curvature_density, dtype=np.float64)
    return cdensity


if __name__ == '__main__':
    pcd = o3d.io.read_point_cloud("../data/1.pcd")
    o3d.visualization.draw_geometries([pcd])

    sc_density = caculate_curvature_density(pcd, radius=0.003)
    mean_sc_density = sc_density.mean()  # 计算曲率密度均值
    ind = np.where((sc_density > mean_sc_density))[0]
    cloud = pcd.select_by_index(ind)
    o3d.visualization.draw_geometries([cloud], window_name="利用点云曲率密度提取特征点",
                                      width=1024, height=768,
                                      left=50, top=50,
                                      mesh_show_back_face=False)

"""
Author: Yixuan Su
Date: 2025/02/19 10:35
File: Open3D 计算点云的面状指数.py
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


def caculate_planarity(cloud, radius=0.003):
    """
    计算点云的面状指数
    :param cloud: 输入点云
    :param radius: k近邻搜索的半径，默认值为：0.003m
    :return: 点云中每个点的面状指数
    """
    points = np.asarray(cloud.points)
    kdtree = o3d.geometry.KDTreeFlann(cloud)
    num_points = len(cloud.points)
    planarity = []  # 储存面状指数
    for i in range(num_points):
        k, idx, _ = kdtree.search_radius_vector_3d(cloud.points[i], radius)

        neighbors = points[idx, :]
        w = pca_compute(neighbors)  # w为特征值
        delt = np.divide(w[1]-w[2], w[0], out=np.zeros_like(w[2]), where=np.sum(w) != 0)
        planarity.append(delt)
    planarity = np.array(planarity, dtype=np.float64)
    return planarity


if __name__ == '__main__':

    pcd = o3d.io.read_point_cloud("../data/bunny.pcd")
    surface_planarity = caculate_planarity(pcd, radius=0.002)
    print(surface_planarity[:10])  # 输出前10个点的面状指数

# ----------------------使用伪颜色显示面状指数-------------------
    planar = np.array(surface_planarity)
    planar_colors = plt.get_cmap('plasma')(
        (planar - planar.min()) / (planar.max() - planar.min()))
    density_colors = planar_colors[:, :3]
    pcd.colors = o3d.utility.Vector3dVector(density_colors)
    o3d.visualization.draw_geometries([pcd], window_name="计算点云面状指数",
                                      width=1024, height=768,
                                      left=50, top=50,
                                      mesh_show_back_face=False)


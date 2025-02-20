"""
Author: Yixuan Su
Date: 2025/02/18 15:24
File: Open3D 计算点云粗糙度（方法一）.py
Description: 

"""
import open3d as o3d
import numpy as np
from matplotlib import pyplot as plt


# 最小二乘拟合平面
def fit_plane_lsq(cloud):
    [center, covariance] = cloud.compute_mean_and_covariance()
    U, S, _ = np.linalg.svd(covariance)
    # 点云最小特征值对应的特征向量即为拟合平面的系数A,B,C
    A = U[0, 2]
    B = U[1, 2]
    C = U[2, 2]
    D = -(A * center[0] + B * center[1] + C * center[2])
    return A, B, C, D


# 计算点到平面的距离
def point_to_plane_dist(point, a, b, c, d):
    dis = abs(a * point[0] + b * point[1] + c *
              point[2] - d) / np.sqrt(a ** 2 + b ** 2 + c ** 2)
    return dis


if __name__ == "__main__":

    # ---------------------------------加载点云----------------------------------
    pcd = o3d.io.read_point_cloud("../data/飞机.pcd")
    # ------------------------------建立KD-tree索引------------------------------
    k = 10
    pcd_tree = o3d.geometry.KDTreeFlann(pcd)
    points = np.asarray(pcd.points)
    number = len(pcd.points)  # 点的个数
    roughness = np.zeros(number)

    # -----------------------------计算每个点的粗糙度----------------------------
    for i in range(number):
        [_, idx, _] = pcd_tree.search_knn_vector_3d(pcd.points[i], k)
        near_cloud = pcd.select_by_index(idx, invert=False)
        [a, b, c, d] = fit_plane_lsq(near_cloud)
        roughness[i] = point_to_plane_dist(points[i], a, b, c, d)

    # ---------------------------使用伪颜色对点云进行渲染-------------------------
    zhot_colors = plt.get_cmap('hot')(  # hot表示为热力图
        (roughness - roughness.min()) / (roughness.max() - roughness.min()))
    zhot_colors = zhot_colors[:, :3]
    pcd.colors = o3d.utility.Vector3dVector(zhot_colors)
    o3d.io.write_point_cloud("render.pcd", pcd)
    o3d.visualization.draw_geometries([pcd], window_name="点云热力图渲染赋色粗糙度",
                                      width=1024, height=768,
                                      left=50, top=50,
                                      mesh_show_back_face=False)


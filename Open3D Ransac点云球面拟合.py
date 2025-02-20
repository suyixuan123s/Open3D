"""
Author: Yixuan Su
Date: 2025/02/18 15:11
File: Open3D Ransac点云球面拟合.py
Description: 

"""

import open3d as o3d
import numpy as np
import sys
import random


def ransac_fit_sphere_process(cloud, dist_th=0.02, max_iters=500):
    """
    Ransac拟合空间球，python详细过程实现
    :param cloud: 输入点云
    :param dist_th: 距离阈值
    :param max_iters: 最大迭代次数
    :return: 球心和半径
    """
    inner = 0  # 内点个数
    sphere_radius = 0  # 球半径
    sphere_center = [0, 0, 0]  # 球心坐标
    sample_num = 4  # 随机选择的点数

    points = np.asarray(cloud.points)
    nums = points.shape[0]
    if sample_num > nums:
        print("拟合球体至少需要四个点!\n")
        sys.exit()

    point_indices = [i for i in range(nums)]
    pcd_tree = o3d.geometry.KDTreeFlann(pcd)
    A = np.zeros([3, 3])
    b = np.zeros(3)

    iters = 0
    while iters < max_iters:
        idx = random.sample(point_indices, sample_num)
        sample_points = points[idx, :]
        # 拟合球
        x = sample_points[:, 0]
        y = sample_points[:, 1]
        z = sample_points[:, 2]
        A[0, 0] = x[0] - x[1]
        A[0, 1] = y[0] - y[1]
        A[0, 2] = z[0] - z[1]
        A[1, 0] = x[0] - x[2]
        A[1, 1] = y[0] - y[2]
        A[1, 2] = z[0] - z[2]
        A[2, 0] = x[0] - x[3]
        A[2, 1] = y[0] - y[3]
        A[2, 2] = z[0] - z[3]
        b[0] = ((x[0] * x[0] - x[1] * x[1]) +
                (y[0] * y[0] - y[1] * y[1]) +
                (z[0] * z[0] - z[1] * z[1])) / 2

        b[1] = ((x[0] * x[0] - x[2] * x[2]) +
                (y[0] * y[0] - y[2] * y[2]) +
                (z[0] * z[0] - z[2] * z[2])) / 2

        b[2] = ((x[0] * x[0] - x[3] * x[3]) +
                (y[0] * y[0] - y[3] * y[3]) +
                (z[0] * z[0] - z[3] * z[3])) / 2

        d = abs(np.linalg.det(A))
        if d < 1e-5:
            continue

        c = np.linalg.inv(A) @ b
        r = np.linalg.norm(c - sample_points[0, :])
        [_, indice1, _] = pcd_tree.search_radius_vector_3d(c, r - dist_th)
        [_, indice2, _] = pcd_tree.search_radius_vector_3d(c, r + dist_th)
        total = len(indice2) - len(indice1)

        if total > inner:
            inner = total
            sphere_center = c
            sphere_radius = r
            # 如果内点数大于总点数的百分之99则停止迭代，0.99即为置信度。
        if inner > 0.99 * nums:
            break
        iters += 1

    if sphere_radius < 1e-5:
        print("拟合失败!!!\n")
        sys.exit()

    return sphere_center, sphere_radius


if __name__ == "__main__":
    pcd = o3d.io.read_point_cloud("E://data//sphere_noisy_oriented.pcd")

    center, radius = ransac_fit_sphere_process(pcd, dist_th=0.2, max_iters=500)
    print("球心坐标：{}\n球半径：{}\n"
          .format(center, radius))

    # 可视化拟合结果
    mesh_circle = o3d.geometry.TriangleMesh.create_sphere(radius=radius)
    mesh_circle.compute_vertex_normals()
    mesh_circle.paint_uniform_color([0.9, 0.1, 0.1])
    mesh_circle = mesh_circle.translate((center[0], center[1], center[2]))
    o3d.visualization.draw_geometries([pcd, mesh_circle])

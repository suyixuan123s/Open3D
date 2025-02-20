"""
Author: Yixuan Su
Date: 2025/02/18 14:59
File: Open3D Ransac拟合空间直线.py
Description: 

"""
import open3d as o3d
import numpy as np
import random


def calculate_rotationMatrix_from_vectors(u, v):
    """
    根据向量计算旋转矩阵
    :param u: 向量u
    :param v: 向量v
    :return: 旋转矩阵
    """
    w = np.cross(u, v)
    c = np.dot(u, v)
    s = np.linalg.norm(w)

    Sx = np.asarray([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])
    R = np.eye(3) + Sx + Sx.dot(Sx) * ((1 - c) / (s ** 2))
    return R


def ransac_fit_3d_line(pts, thresh=0.2, maxIteration=1000):
    """
    求出三维直线的最佳方程。三维环境中的直线定义为y = Ax+B，但A和B是向量而不是标量。

    :param pts: `np.array (N,3)`类型的3D点云.
    :param thresh: 距离阈值
    :param maxIteration: RANSAC最大迭代次数
    :returns:
    - `A`: 空间直线的斜率 `np.array (1, 3)`
    - `B`: 空间直线的截距`np.array (1, 3)`
    - `inliers`: 内点索引 `np.array (1, M)`
    ---
    """
    inliers = []
    A = []
    B = []

    n_points = pts.shape[0]
    best_inliers = []

    for it in range(maxIteration):

        # 随机选取两个点
        id_samples = random.sample(range(0, n_points), 2)
        pt_samples = pts[id_samples]
        # The line defined by two points is defined as P2 - P1
        vecA = pt_samples[1, :] - pt_samples[0, :]
        vecA_norm = vecA / np.linalg.norm(vecA)
        # Distance from a point to a line
        vecC_stakado = np.stack([vecA_norm] * n_points, 0)
        dist_pt = np.cross(vecC_stakado, (pt_samples[0, :] - pts))
        dist_pt = np.linalg.norm(dist_pt, axis=1)
        # Select indexes where distance is biggers than the threshold
        pt_id_inliers = np.where(np.abs(dist_pt) <= thresh)[0]

        if len(pt_id_inliers) > len(best_inliers):
            best_inliers = pt_id_inliers
            inliers = best_inliers
            A = vecA_norm
            B = pt_samples[0, :]

    return A, B, inliers


pcd = o3d.io.read_point_cloud("../data/飞机.pcd")
pcd.paint_uniform_color([0, 0, 1])
print('获取输入数据的三维坐标')
points = np.asarray(pcd.points)
# --------------------------------RANSAC拟合直线--------------------------------------
print('进行RANSAC拟合直线，拟合结果为：')
# A:直线的斜率，B：直线的截距，inliers：内点索引，
# thresh：内点的距离阈值
# maxIteration：RANSAC算法的拟合次数
A, B, inliers = ransac_fit_3d_line(points, thresh=0.05, maxIteration=500)
print('直线的三维斜率为：', A)
print('直线的截距为：', B)
R = calculate_rotationMatrix_from_vectors([0, 0, 1], A)
ransac_line = pcd.select_by_index(inliers)
# -----------------------------------计算点云最值-------------------------------------
max_pts = pcd.get_max_bound()
print("点云坐标最大值为：", max_pts)
min_pts = pcd.get_min_bound()
print("点云坐标最小值为：", min_pts)
line_length = np.linalg.norm(max_pts - min_pts)
# -----------------------------------结果可视化----------------------------------------
mesh_cylinder = o3d.geometry.TriangleMesh.create_cylinder(radius=0.02, height=line_length * 2)
mesh_cylinder.compute_vertex_normals()
mesh_cylinder.paint_uniform_color([1, 0, 0])
mesh_cylinder = mesh_cylinder.rotate(R, center=[0, 0, 0])
mesh_cylinder = mesh_cylinder.translate((B[0], B[1], B[2]))
o3d.visualization.draw_geometries([pcd, ransac_line, mesh_cylinder])

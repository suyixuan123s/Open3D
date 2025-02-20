"""
Author: Yixuan Su
Date: 2025/02/16 18:00
File: 附有限制条件的间接平差拟合空间圆.py
Description: 

"""
import numpy as np
import open3d as o3d


def circle_fit_3d_points_m(points):
    """
    全局最小二乘拟合空间直线
    :param points: 三维点集合
    :return: 直线参数向量数组X
    """
    n = points.shape[0]  # 点的个数
    A = points
    l = np.ones((n, 1))  #
    NAA = np.dot(A.T, A)
    WA = np.dot(A.T, l)
    AX = np.linalg.inv(NAA).dot(WA)

    B = np.zeros((n - 1, 3))  # 矩阵B
    L = np.zeros((n - 1, 1))  # 矩阵L
    # 构建计算所需矩阵
    for i in range(0, n - 1):
        B[i, :] = A[i + 1, :] - A[i, :]
        L[i] = ((np.linalg.norm(A[i + 1, :])) ** 2 - (np.linalg.norm(A[i, :])) ** 2) * 0.5

    C = [AX[0], AX[1], AX[2]]

    N = np.zeros((4, 4))
    NBB = B.T.dot(B)
    N[:3, :3] = NBB
    N[0, 3] = C[0]
    N[1, 3] = C[1]
    N[2, 3] = C[2]
    N[3, 0] = C[0]
    N[3, 1] = C[1]
    N[3, 2] = C[2]
    W = np.ones((4, 1))  # 解X
    W[0:3] = B.T.dot(L)
    Xo = np.linalg.inv(N).dot(W)
    center = np.array([Xo[0], Xo[1], Xo[2]])

    r = np.linalg.norm(A - center.T, axis=-1)
    r = r.mean()

    normalized_v = np.linalg.norm(AX)
    normalized_v = AX / normalized_v

    return center, normalized_v.T, r


# -------------------------加载点云--------------------------------
pcd = o3d.io.read_point_cloud('../../../data/Armadillo.pcd')
points = np.asarray(pcd.points)
[x0, y0, z0] = np.mean(points, axis=0)  # 点云质心
x = points[:, 0]
y = points[:, 1]
z = points[:, 2]
N = points.shape[0]
center1, normal, radius = circle_fit_3d_points_m(points)
print("center: \n" + str(center1))
print("radius: \n" + str(radius))
print("vecC: \n" + str(normal))


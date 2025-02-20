"""
Author: Yixuan Su
Date: 2025/02/18 11:02
File: Open3D 最小二乘拟合空间直线（方法三）.py
Description: 

"""
import numpy as np
import matplotlib.pyplot as plt
import open3d as o3d


def line_fit_3d_points(points):
    """
    最小二乘拟合空间直线
    :param points: 三维点集合
     直线方程：
     x = a * z + b
     y = c * z + d
    :return: 直线参数a,b,c,d
    """
    x = points[:, 0]
    y = points[:, 1]
    z = points[:, 2]
    n = points.shape[0]
    a = (n * sum(x * z) - sum(x) * sum(z)) / (n * sum(z * z) - sum(z) * sum(z))
    b = (sum(x) - a * sum(z)) / n
    c = (n * sum(y * z) - sum(y) * sum(z)) / (n * sum(z * z) - sum(z) * sum(z))
    d = (sum(y) - c * sum(z)) / n

    return a, b, c, d


def line_fit_3d_points_m(points):
    """
    全局最小二乘拟合空间直线
    :param points: 三维点集合
     直线方程：
     x = a * z + b
     y = c * z + d
    :return: 直线参数向量数组X
    """

    n = points.shape[0]  # 点的个数
    B = np.zeros((n * 2, 4))  # 矩阵B
    L = np.zeros((n * 2, 1))  # 矩阵L
    X = np.ones((4, 1))  # 解X
    deltX = np.ones((4, 1))  # 迭代增量deltX
    # 构建计算所需矩阵
    for i in range(0, n):
        B[2 * i, 0] = points[i][2]
        B[2 * i, 1] = 1
        B[2 * i + 1, 2] = points[i][2]
        B[2 * i + 1, 3] = 1

        L[2 * i] = points[i][0]
        L[2 * i + 1] = points[i][1]
    # 迭代计算
    count = 0
    while np.linalg.norm(deltX) > 0.0001:
        l = L - B.dot(X)
        NBB = B.T.dot(B)
        W = B.T.dot(l)
        deltX = np.linalg.inv(NBB).dot(W)

        X += deltX
        count += 1
        if count >= 10:
            break
    print("迭代次数:", count)

    return X


def line_fit_3d_points_n(points):
    """
    最小二乘拟合空间直线
    :param points: 三维点集合
     直线方程：
     x = a * z + b
     y = c * z + d
    :return: 直线参数
    """
    x = points[:, 0]
    y = points[:, 1]
    n = points.shape[0]
    M = np.ones((2, n))
    M[0, :] = points[:, 2]
    A = np.linalg.inv(M.dot(M.T)).dot(M.dot(x))
    B = np.linalg.inv(M.dot(M.T)).dot(M.dot(y))

    X = np.array([A[0], A[1], B[0], B[1]])

    return X


# -------------------------加载点云--------------------------------
pcd = o3d.io.read_point_cloud('../data/1.pcd')
points = np.asarray(pcd.points)
[x0, y0, z0] = np.mean(points, axis=0)  # 点云质心
x = points[:, 0]
y = points[:, 1]
z = points[:, 2]
N = points.shape[0]
res = line_fit_3d_points_n(points)
print("a=", res[0])
print("b=", res[1])
print("c=", res[2])
print("d=", res[3])
# --------------------------可视化拟合结果----------------------------
fig1 = plt.figure()
ax1 = fig1.add_subplot(111, projection='3d')
ax1.set_xlabel("x")
ax1.set_ylabel("y")
ax1.set_zlabel("z")
ax1.scatter(x, y, z, c='r', marker='o')

x1 = res[0] * z + res[1]
y1 = res[2] * z + res[3]
ax1.plot(x1, y1, z, c='blue', ls='-')
plt.title("Least squares fit space straight lines")
plt.show()


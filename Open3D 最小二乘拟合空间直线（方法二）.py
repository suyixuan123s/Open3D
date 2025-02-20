"""
Author: Yixuan Su
Date: 2025/02/18 11:11
File: Open3D 最小二乘拟合空间直线（方法二）.py.py
Description: 

"""
import numpy as np
import matplotlib.pyplot as plt
import open3d as o3d


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

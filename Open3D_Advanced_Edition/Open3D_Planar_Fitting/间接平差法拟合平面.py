"""
Author: Yixuan Su
Date: 2025/02/16 17:26
File: 间接平差法拟合平面.py
Description: 

"""
import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt



def pingcha_fit_plane(pts):
    """
    间接平差法拟合平面
    :param points: 三维点集合
    :return: 平面法向量res
    """
    n = pts.shape[0]  # 点的个数
    B = np.ones((n, 3))  #
    l = np.ones((n, 1))  #
    # 构建计算所需矩阵
    for i in range(0, n):
        B[i, 0] = pts[i][0]
        B[i, 1] = pts[i][1]
        l[i] = pts[i][2]

    NBB = np.dot(B.T, B)
    NBB_inv = np.linalg.inv(NBB)
    W = np.dot(B.T, l)
    X = NBB_inv.dot(W)

    return X


# -------------------------加载点云--------------------------------
pcd = o3d.io.read_point_cloud('../../../data/bunny.pcd')
points = np.asarray(pcd.points)
x = points[:, 0]
y = points[:, 1]
z = points[:, 2]

# 求解
X = pingcha_fit_plane(points)
print('平面拟合结果为：z = %.3f * x + %.3f * y + %.3f' % (X[0], X[1], X[2]))
# -------------------------结果展示-------------------------------
fig1 = plt.figure()
ax1 = fig1.add_subplot(111, projection='3d')
ax1.set_xlabel("x")
ax1.set_ylabel("y")
ax1.set_zlabel("z")
min_pt = np.amin(points, axis=0)  # 获取坐标最小值
max_pt = np.amax(points, axis=0)  # 获取坐标最大值
ax1.scatter(x, y, z, c='r', marker='o')
x_p = np.linspace(min_pt[0], max_pt[0], 100)
y_p = np.linspace(min_pt[1], max_pt[1], 100)
x_p, y_p = np.meshgrid(x_p, y_p)
z_p = X[0] * x_p + X[1] * y_p + X[2]
ax1.plot_wireframe(x_p, y_p, z_p, rstride=10, cstride=10)
plt.title("Global least squares fit plane")
plt.show()


"""
Author: Yixuan Su
Date: 2025/02/18 11:27
File: Open3D 间接平差拟合二维圆.py
Description: 

"""
import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt

# -------------------------------------加载点云-------------------------------------
pcd = o3d.io.read_point_cloud("../data/1.pcd")
points = np.asarray(pcd.points)
x = points[:, 0]
y = points[:, 1]
n = points.shape[0]  # 点的个数
# ----------------------------------间接平差拟合圆-----------------------------------

B = np.ones((n, 3))  #
L = np.ones((n, 1))  #
# 构建计算所需矩阵
B[:, 0] = x * 2
B[:, 1] = y * 2
L[:, 0] = x ** 2 + y ** 2

NBB = np.dot(B.T, B)
NBB_inv = np.linalg.inv(NBB)
W = np.dot(B.T, L)
X = NBB_inv.dot(W)
# 拟合圆的半径
r0 = np.sqrt(X[0] ** 2 + X[1] ** 2 + X[2])
print(X[0])
# -------------------------------------结果展示------------------------------------
plt.figure(facecolor='white')  # figsize=(7, 5.4), dpi=72,
plt.axis('equal')
# 标题
plt.title('Least Squares Circle')

theta_fit = np.linspace(-np.pi, np.pi, 180)

x_fit2 = X[0] + r0 * np.cos(theta_fit)
y_fit2 = X[1] + r0 * np.sin(theta_fit)
plt.plot(x_fit2, y_fit2, 'k--', label='Fitting Circle', lw=2)

plt.plot([X[0]], [X[1]], 'gD', mec='r', mew=1)
# draw
plt.xlabel('x')
plt.ylabel('y')
# 数据
plt.plot(x, y, 'ro', label='data', ms=8, mec='b', mew=1)
plt.legend(loc='best', labelspacing=0.1)
plt.show()

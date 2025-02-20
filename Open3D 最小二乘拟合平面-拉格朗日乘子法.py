"""
Author: Yixuan Su
Date: 2025/02/18 15:04
File: Open3D 最小二乘拟合平面-拉格朗日乘子法.py
Description: 

"""
import numpy as np
import matplotlib.pyplot as plt
import open3d as o3d

# -------------------------加载点云--------------------------------
pcd = o3d.io.read_point_cloud('../data/飞机.pcd')
points = np.asarray(pcd.points)
[x0, y0, z0] = np.mean(points, axis=0)  # 点云质心
x = points[:, 0]
y = points[:, 1]
z = points[:, 2]
N = points.shape[0]
xyz = points - [x0, y0, z0]  # 去质心
x1 = xyz[:, 0]
y1 = xyz[:, 1]
z1 = xyz[:, 2]
# ------------------------构建系数矩阵-----------------------------
A = np.array([[sum(x * x), sum(x * y), sum(x * z)],
              [sum(x * y), sum(y * y), sum(y * z)],
              [sum(x * z), sum(y * z), sum(z * z)]])
[D, U] = np.linalg.eig(A)
# 点云最小特征值对应的特征向量即为拟合平面的系数A,B,C
A = U[0, 2]
B = U[1, 2]
C = U[2, 2]
D = -(A * x0 + B * y0 + C * z0)
# --------------------------可视化拟合结果----------------------------
fig1 = plt.figure()
ax1 = fig1.add_subplot(111, projection='3d')
ax1.set_xlabel("x")
ax1.set_ylabel("y")
ax1.set_zlabel("z")
min_pt = np.amin(points, axis=0)  # 获取坐标最小值
max_pt = np.amax(points, axis=0)  # 获取坐标最大值
ax1.scatter(x, y, z, c='r', marker='^')
x_p = np.linspace(min_pt[0], max_pt[0], 100)
y_p = np.linspace(min_pt[1], max_pt[1], 100)
XFit, YFit = np.meshgrid(x_p, y_p)

ZFit = -(D + A * XFit + B * YFit) / C
ax1.plot_wireframe(XFit, YFit, ZFit, rstride=10, cstride=10)
plt.show()

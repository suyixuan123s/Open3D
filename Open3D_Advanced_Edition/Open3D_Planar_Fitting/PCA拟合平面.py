"""
Author: Yixuan Su
Date: 2025/02/16 17:36
File: PCA拟合平面.py
Description: 

"""
import numpy as np
import open3d as o3d
from sklearn.decomposition import PCA
import matplotlib.pyplot as plt

# ----------------------------加载点云数据---------------------------------
pcd = o3d.io.read_point_cloud("../../../data/bunny.pcd")
X = np.asarray(pcd.points)
# -----------------------------调用PCA------------------------------------
pca = PCA(n_components=3)  # 设置保留主成分个数
pca.fit(X)
eigenVector = pca.components_  # 按行排列，第一主成分排在首行
centrid = pca.mean_
# print(eigenVector)
# 点云最小特征值对应的特征向量即为拟合平面的系数A,B,C
A = eigenVector[2, 0]
B = eigenVector[2, 1]
C = eigenVector[2, 2]
D = -(A * centrid[0] + B * centrid[1] + C * centrid[2])
# ---------------------------输出拟合结果----------------------------------
print('平面拟合结果为：%.6f * x + %.6f * y + %.6f*z + %.6f = 0' % (A, B, C, D))
# --------------------------可视化拟合结果---------------------------------
fig1 = plt.figure()
ax1 = fig1.add_subplot(111, projection='3d')
ax1.set_title('PCA Fit Plane')
ax1.set_xlabel("x")
ax1.set_ylabel("y")
ax1.set_zlabel("z")
# 获取xyz坐标及最值用于plot绘图
points = np.asarray(pcd.points)  # 获取点坐标
min_pt = np.amin(points, axis=0)  # 获取坐标最小值
max_pt = np.amax(points, axis=0)  # 获取坐标最大值
ax1.scatter(points[:, 0], points[:, 1], points[:, 2], c='r', marker='^')
x_p = np.linspace(min_pt[0], max_pt[0], 100)
y_p = np.linspace(min_pt[1], max_pt[1], 100)
XFit, YFit = np.meshgrid(x_p, y_p)

ZFit = -(D + A * XFit + B * YFit) / C
ax1.plot_wireframe(XFit, YFit, ZFit, rstride=10, cstride=10)
plt.show()


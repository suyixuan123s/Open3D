"""
Author: Yixuan Su
Date: 2025/02/18 15:55
File: Open3D 最小二乘拟合二维直线.py
Description: 

"""
import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt

# -------------------------------------加载点云-------------------------------------
pcd = o3d.io.read_point_cloud("../data/飞机1.pcd")
points = np.asarray(pcd.points)
x = points[:, 0]
y = points[:, 1]
# -----------------------------------拟合直线y=kx+c--------------------------------
A = np.vstack([x, np.ones(len(x))]).T
k, c = np.linalg.lstsq(A, y, rcond=None)[0]
print("直线系数为\nk=：{}\nb=：{}\n"
      .format(k, c))
error = np.linalg.lstsq(A, y, rcond=None)[1]  # 残差平方和:每列的欧几里得2范数的平方
print('残差平方和:', error)
rank = np.linalg.lstsq(A, y, rcond=None)[2]  # 矩阵a的秩
print('矩阵a的秩：', rank)
s = np.linalg.lstsq(A, y, rcond=None)[3]  # 矩阵a的奇异值
print('矩阵a的奇异值：', s)
plt.plot(x, y, 'o', label='Original data', markersize=3)
plt.plot(x, k * x + c, 'r', label='Fitted line')
plt.legend()
plt.show()

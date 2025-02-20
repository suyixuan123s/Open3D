"""
Author: Yixuan Su
Date: 2025/02/18 15:02
File: Open3D 最小二乘拟合二维直线（直接求解法）.py
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
# -----------------------------------拟合直线y=kx+b--------------------------------
n = points.shape[0]
xm = np.mean(x)
ym = np.mean(y)
k = (sum(x * y) - n * xm * ym) / (sum(x * x) - n * xm * xm)
b = ym - k * xm
print(k)
print(b)
plt.plot(x, y, 'o', label='Original data', markersize=3)
plt.plot(x, k * x + b, 'r', label='Fitted line')
plt.legend()
plt.show()

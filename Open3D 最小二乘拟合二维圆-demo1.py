"""
Author: Yixuan Su
Date: 2025/02/18 16:03
File: Open3D 最小二乘拟合二维圆-demo1.py
Description: 

"""
import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt

# -------------------------------------加载点云-------------------------------------
pcd = o3d.io.read_point_cloud("../data/飞机.pcd")
# ---------------------------------最小二乘拟合二维圆--------------------------------
points = np.asarray(pcd.points)
n = points.shape[0]  # 点的个数
x = points[:, 0]  # x坐标
y = points[:, 1]  # y坐标
# 构建相关系数
A = np.column_stack((x, y, np.ones([n, 1])))
b = -(x ** 2 + y ** 2)
# 求解线性方程
at = np.linalg.pinv(A)
X = at.dot(b)
# 计算圆心和半径
o_x = -0.5 * X[0]
o_y = -0.5 * X[1]
r = np.sqrt(o_x ** 2 + o_y ** 2 - X[2])
center = [o_x, o_y]
circleRadius = r
print("圆心坐标：{}\n半径：{}\n"
      .format(center, circleRadius))
# --------------------------------结果可视化---------------------------------
a = center[0]
b = center[1]
theta = np.arange(0, 2 * np.pi, 0.01)
x = a + circleRadius * np.cos(theta)
y = b + circleRadius * np.sin(theta)
fig, ax = plt.subplots()
ax.set_title('LS Fit 2D Circle')
ax.scatter(points[:, 0], points[:, 1], color=[0, 1, 0])
ax.plot(x, y, color=[1, 0, 0], linewidth=1)
ax.axis('scaled')  # 等比横纵坐标
plt.show()

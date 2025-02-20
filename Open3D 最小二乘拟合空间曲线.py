"""
Author: Yixuan Su
Date: 2025/02/19 16:17
File: Open3D 最小二乘拟合空间曲线.py
Description: 

"""
import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt

# -------------------------------------加载点云-------------------------------------
pcd = o3d.io.read_point_cloud("../data/1.pcd")
points = np.asarray(pcd.points)
x = points[:, 0]
y = points[:, 1]
z = points[:, 2]
n = points.shape[0]  # 点的个数
# ------------------------------------拟合空间曲线----------------------------------

B = np.ones((n, 3))  #
L = np.ones((n, 1))  #
# 构建计算所需矩阵
B[:, 0] = x ** 2 + y ** 2
B[:, 1] = np.sqrt(x ** 2 + y ** 2)
L[:, 0] = z
# 最小二乘求解
X = np.linalg.lstsq(B, L, rcond=None)
# 空间曲线的参数
print('空间曲线参数为：\n', X[0])
# # -------------------------------------结果展示------------------------------------
# 创建图形和三维坐标轴
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# 绘制三维线图
ax.plot(x, y, z, label='3D Cure')

# 设置坐标轴标签
ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')

# 设置标题
plt.title('3D Cure Plot')

# 显示图形
plt.show()

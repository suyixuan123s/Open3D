"""
Author: Yixuan Su
Date: 2025/02/18 15:00
File: Open3D 最小二乘拟合平面（SVD分解法）.py
Description: 

"""
import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt

# -----------------------------读取点云--------------------------------
pcd = o3d.io.read_point_cloud("../data/飞机.pcd")
# ----------------------------可视化点云-------------------------------
o3d.visualization.draw_geometries([pcd], window_name="显示点云",
                                  width=1024, height=768,
                                  left=50, top=50,
                                  mesh_show_back_face=False)
# -----------------使用SVD分解进行最小二乘拟合平面-----------------------
center = pcd.get_center()  # 计算点云质心
points = np.asarray(pcd.points)  # 获取点坐标
H = points - center  # 去质心
U, S, V = np.linalg.svd(H)  # 矩阵奇异值分解

# 点云最小特征值对应的特征向量即为拟合平面的系数A,B,C
A = V[2, 0]
B = V[2, 1]
C = V[2, 2]
D = -(A * center[0] + B * center[1] + C * center[2])
# 输出拟合结果
print('平面拟合结果为：%.6f * x + %.6f * y + %.6f*z + %.6f = 0' % (A, B, C, D))
# --------------------------可视化拟合结果----------------------------
fig1 = plt.figure()
ax1 = fig1.add_subplot(111, projection='3d')
ax1.set_title('SVD Fit Plane')
ax1.set_xlabel("x")
ax1.set_ylabel("y")
ax1.set_zlabel("z")
# 获取xyz坐标及最值用于plot绘图

min_pt = np.amin(points, axis=0)  # 获取坐标最小值
max_pt = np.amax(points, axis=0)  # 获取坐标最大值
ax1.scatter(points[:, 0], points[:, 1], points[:, 2], c='r', marker='.')
x_p = np.linspace(min_pt[0], max_pt[0], 100)
y_p = np.linspace(min_pt[1], max_pt[1], 100)
XFit, YFit = np.meshgrid(x_p, y_p)

ZFit = -(D + A * XFit + B * YFit) / C
ax1.plot_wireframe(XFit, YFit, ZFit, rstride=10, cstride=10)
plt.show()

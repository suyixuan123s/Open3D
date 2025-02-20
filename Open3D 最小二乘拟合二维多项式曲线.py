"""
Author: Yixuan Su
Date: 2025/02/18 15:52
File: Open3D 最小二乘拟合二维多项式曲线.py
Description: 

"""

import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt

# -------------------------------读取点云--------------------------------------
pcd = o3d.io.read_point_cloud("../data/飞机.pcd")
# ----------------------------定义x、y散点坐标---------------------------------
points = np.asarray(pcd.points)
x = points[:, 0]
y = points[:, 1]
# 用7次多项式拟合
f = np.polyfit(x, y, 7)  # 线性最小二乘拟合
p = np.poly1d(f)
print(p)  # 打印出拟合函数
yvals1 = p(x)  # 拟合y值
# --------------------------------结果可视化-----------------------------------
plot1 = plt.plot(x, y, 'bo', mec='b', mew=1, label='original values')
plot2 = plt.plot(x, yvals1, 'r', label='polyfit values')

plt.xlabel('x')
plt.ylabel('y')
plt.legend(loc=2, bbox_to_anchor=(1.05, 1.0), borderaxespad=0.)
plt.title('polyfitting')
# plt.savefig('nihe1.png')
plt.show()

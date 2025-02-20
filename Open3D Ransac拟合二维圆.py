"""
Author: Yixuan Su
Date: 2025/02/18 16:00
File: Open3D Ransac拟合二维圆.py
Description: 

"""
import open3d as o3d
import numpy as np
import random
import matplotlib.pyplot as plt

# -------------------------------------加载点云-------------------------------------
pcd = o3d.io.read_point_cloud("../data/飞机.pcd")
# -------------------------------------参数设置-------------------------------------
maxIters = 100  # 迭代次数
distTh = 0.01  # 距离阈值
sample_num = 3  # 随机选择的点数
total = 0
inner = 0  # 内点个数
circleRadius = 0  # 半径
center = np.array([0, 0])  # 圆心
points = np.asarray(pcd.points)
points = points[:, 0:2]
nums = points.shape[0]
pointIdx = [i for i in range(nums)]
# ---------------------------------RANSAC纤细过程-----------------------------------
iters = 0
while iters < maxIters:
    idx = random.sample(pointIdx, sample_num)
    sample_data = points[idx, :]
    p1 = points[idx[0], :]
    p2 = points[idx[1], :]
    p3 = points[idx[2], :]

    x1 = p1[0]
    y1 = p1[1]
    x2 = p2[0]
    y2 = p2[1]
    x3 = p3[0]
    y3 = p3[1]
    # 判断三点是否共线，如果共线则重新选取
    if (x2 - x1) * (y3 - y1) == (x3 - x1) * (y2 - y1):
        continue

    # 计算圆心和半径
    x0 = ((y2 - y1) * (y3 * y3 - y1 * y1 + x3 * x3 - x1 * x1) - (y3 - y1) * (y2 * y2 - y1 * y1 + x2 * x2 - x1 * x1)) \
         / (2 * (x3 - x1) * (y2 - y1) - 2 * (x2 - x1) * (y3 - y1))
    y0 = ((x2 - x1) * (x3 * x3 - x1 * x1 + y3 * y3 - y1 * y1) - (x3 - x1) * (x2 * x2 - x1 * x1 + y2 * y2 - y1 * y1)) \
         / (2 * (y3 - y1) * (x2 - x1) - 2 * (y2 - y1) * (x3 - x1))

    r = np.sqrt(pow((x0 - x1), 2) + pow((y0 - y1), 2))

    sc = np.array([x0, y0])
    tmpr = np.sqrt(np.sum((points - sc) ** 2, axis=1))  # 计算每个数据到圆心的距离
    if (abs(tmpr - r) < distTh).all():
        total += 1

    if total > inner:
        inner = total
        center = sc
        circleRadius = r
    # 如果内点数大于总点数的百分之99则停止迭代，0.99即为置信度。
    if inner > 0.99 * nums:
        break
    iters += 1
print("圆心坐标：{}\n半径：{}\n内部点数为：{}\n"
      .format(center, circleRadius, inner))

# --------------------------------结果可视化---------------------------------
a = center[0]
b = center[1]
theta = np.arange(0, 2 * np.pi, 0.01)
x = a + circleRadius * np.cos(theta)
y = b + circleRadius * np.sin(theta)
fig, ax = plt.subplots()
ax.set_title('Ransac Fit 2D Circle')
ax.scatter(points[:, 0], points[:, 1], color=[0, 1, 0])
ax.plot(x, y, color=[1, 0, 0], linewidth=1)
ax.axis('scaled')  # 等比横纵坐标
plt.show()

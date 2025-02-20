"""
Author: Yixuan Su
Date: 2025/02/18 15:56
File: Open3D Ransac拟合二维直线.py
Description:

"""
import open3d as o3d
import numpy as np
import random
import matplotlib.pyplot as plt


def fit_2d_line_by_ransac(cloud, max_iters=100, dist=0.01):
    """
    Ransac拟合二维直线
    :param cloud: 输入点云
    :param max_iters: 最大迭代次数
    :param dist: 距离阈值
    :return: 直线y=kx+b的系数k,b
    """
    sample_num = 2  # 随机选择的点数
    inner = 0  # 内点个数
    line_coff = np.array([0, 0])  # 直线参数
    points = np.asarray(cloud.points)
    nums = points.shape[0]
    pointIdx = [i for i in range(nums)]
    # ---------------------------------RANSAC详细过程-----------------------------------
    iters = 0
    while iters < max_iters:
        total = 0  # 内点计数器清零
        idx = random.sample(pointIdx, sample_num)
        p1 = points[idx[0], :]
        p2 = points[idx[1], :]

        x0 = p1[0]
        y0 = p1[1]
        x1 = p2[0]
        y1 = p2[1]
        # 判断两个点是否重合
        if x0 == x1:
            continue
        # 斜率和截距
        k = (y1 - y0) / (x1 - x0)
        b = y1 - k * x1
        # 算出内点数目
        for idx in range(nums):
            y_estimate = k * points[idx][0] + b
            if abs(y_estimate - points[idx][1]) < dist:
                total += 1
        # 如果本次循环内点比上次多，则采用本次
        if total > inner:
            inner = total
            line_coff = np.asarray([k, b])
        # 如果内点数大于总点数的百分之99则停止迭代，0.99即为置信度。
        if inner > 0.99 * nums:
            break
        iters += 1

    return line_coff  # 将return语句放到while循环外面，确保完成所有迭代后再返回

#
#
# def fit_2d_line_by_ransac(cloud, max_iters=100, dist=0.01):
#     """
#     Ransac拟合二维直线
#     :param cloud: 输入点云
#     :param max_iters: 最大迭代次数
#     :param dist: 距离阈值
#     :return: 直线y=kx+b的系数k,b
#     """
#     sample_num = 2  # 随机选择的点数
#     total = 0
#     inner = 0  # 内点个数
#     line_coff = np.array([0, 0])  # 直线参数
#     points = np.asarray(cloud.points)
#     nums = points.shape[0]
#     pointIdx = [i for i in range(nums)]
#     # ---------------------------------RANSAC详细过程-----------------------------------
#     iters = 0
#     while iters < max_iters:
#         idx = random.sample(pointIdx, sample_num)
#         p1 = points[idx[0], :]
#         p2 = points[idx[1], :]
#
#         x0 = p1[0]
#         y0 = p1[1]
#         x1 = p2[0]
#         y1 = p2[1]
#         # 判断两个点是否重合
#         if x0 == x1:
#             continue
#         # 斜率和截距
#         k = (y1 - y0) / (x1 - x0)
#         b = y1 - k * x1
#         # 算出内点数目
#         for idx in range(nums):
#             y_estimate = k * points[idx][0] + b
#             if abs(y_estimate - points[idx][1]) < dist:
#                 total += 1
#         # 如果本次循环内点比上次多，则采用本次
#         if total > inner:
#             inner = total
#             line_coff = np.asarray([k, b])
#         # 如果内点数大于总点数的百分之99则停止迭代，0.99即为置信度。
#         if inner > 0.99 * nums:
#             break
#         iters += 1
#         return line_coff


# -------------------------------------加载点云-------------------------------------
pcd = o3d.io.read_point_cloud("../data/飞机.pcd")
# -------------------------------------参数设置-------------------------------------
maxIters = 100  # 迭代次数
distTh = 0.2  # 距离阈值
line_parm = fit_2d_line_by_ransac(pcd, maxIters, distTh)
print("直线系数为\nk=：{}\nb=：{}\n"
      .format(line_parm[0], line_parm[1]))
# -------------------------------------结果可视化-----------------------------------
show_points = np.asarray(pcd.points)
x = show_points[:, 0]
y = show_points[:, 1]
m = line_parm[0]
c = line_parm[1]
plt.plot(x, y, 'o', label='Original data', markersize=3)
plt.plot(x, m * x + c, 'r', label='Fitted line')
plt.legend()
plt.show()

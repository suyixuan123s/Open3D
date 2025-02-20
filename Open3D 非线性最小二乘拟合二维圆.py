"""
Author: Yixuan Su
Date: 2025/02/18 15:30
File: Open3D 非线性最小二乘拟合二维圆.py
Description: 

"""
import open3d as o3d
import numpy as np
import scipy.optimize as opt
import matplotlib.pyplot as plt


def circle2d_function(point, a, b, r):
    """
    构造圆拟合函数，计算以a,b,r为参数的圆和原始数据之间的误差
    """
    x = point[0, :]
    y = point[1, :]
    func = pow((x - a), 2) + pow((y - b), 2) - pow(r, 2)
    return func


def circle2d_fit(point):
    """"非线性最小二乘拟合"""
    best = np.zeros(point.shape[0])
    popt, pcov = opt.curve_fit(circle2d_function, point.T, best, p0=[4.0, 1.4, -4.5])
    circle2d_r = abs(popt[2])
    circle2d_o = [popt[0], popt[1]]
    return circle2d_o, circle2d_r


if __name__ == '__main__':
    pcd = o3d.io.read_point_cloud("../data/depth2cloud.pcd")
    point3d = np.asarray(pcd.points)
    center, R = circle2d_fit(point3d)
    print("圆心坐标:%s" % center)
    print("半径:%s" % R)
    # --------------------------------结果可视化---------------------------------
    a = center[0]
    b = center[1]
    theta = np.arange(0, 2 * np.pi, 0.01)
    x = a + R * np.cos(theta)
    y = b + R * np.sin(theta)
    fig, ax = plt.subplots()
    ax.set_title('Curve LS Fit 2D Circle')
    ax.plot(point3d[:, 0], point3d[:, 1], 'ro', mec='r', mew=1)
    ax.plot(x, y, 'k--', label='Fitting Circle', lw=2)
    ax.axis('scaled')  # 等比横纵坐标
    plt.show()

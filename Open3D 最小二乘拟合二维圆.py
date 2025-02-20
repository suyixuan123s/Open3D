"""
Author: Yixuan Su
Date: 2025/02/18 15:53
File: Open3D 最小二乘拟合二维圆.py
Description: 

"""
import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
from scipy import optimize
import functools

# -------------------------------------加载点云-------------------------------------
pcd = o3d.io.read_point_cloud("../data/飞机1.pcd")
points = np.asarray(pcd.points)
x = points[:, 0]
y = points[:, 1]


# ------------------------------------最小二乘拟合-----------------------------------
# 修饰器：用于输出反馈
def countcalls(fn):
    "decorator function count function calls "

    @functools.wraps(fn)
    def wrapped(*args):
        wrapped.ncalls += 1
        return fn(*args)

    wrapped.ncalls = 0
    return wrapped


def calc_r(xc, yc):
    return np.sqrt((x - xc) ** 2 + (y - yc) ** 2)


@countcalls
def f_2(c):
    Ri = calc_r(*c)
    return Ri - Ri.mean()


# 质心坐标
x_m = np.mean(x)
y_m = np.mean(y)
# 圆心估计
center_estimate = x_m, y_m
center_2, _ = optimize.leastsq(f_2, center_estimate)

xc_2, yc_2 = center_2
Ri_2 = calc_r(xc_2, yc_2)
# 拟合圆的半径
R_2 = Ri_2.mean()
residu_2 = sum((Ri_2 - R_2) ** 2)
residu2_2 = sum((Ri_2 ** 2 - R_2 ** 2) ** 2)
ncalls_2 = f_2.ncalls
# -------------------------------------结果展示------------------------------------
plt.figure(facecolor='white')  # figsize=(7, 5.4), dpi=72,
plt.axis('equal')
# 标题
plt.title('Least Squares Circle')

theta_fit = np.linspace(-np.pi, np.pi, 180)

x_fit2 = xc_2 + R_2 * np.cos(theta_fit)
y_fit2 = yc_2 + R_2 * np.sin(theta_fit)
plt.plot(x_fit2, y_fit2, 'k--', label='Fitting Circle', lw=2)

plt.plot([xc_2], [yc_2], 'gD', mec='r', mew=1)
# draw
plt.xlabel('x')
plt.ylabel('y')
# 数据
plt.plot(x, y, 'ro', label='data', ms=8, mec='b', mew=1)
plt.legend(loc='best', labelspacing=0.1)
plt.show()

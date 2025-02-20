"""
Author: Yixuan Su
Date: 2025/02/18 15:50
File: Open3D 非线性最小二乘拟合二维多项式曲线.py
Description: 

"""
import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit


def func(x, a, b, c, d, e, f, g, h):
    return a + b * x + c * x ** 2 + d * pow(x, 3) + e * pow(x, 4) + f * pow(x, 5) + g * pow(x, 6) + h * pow(x, 7)


# 定义x、y散点坐标
# -------------------------------读取点云--------------------------------------
pcd = o3d.io.read_point_cloud("../data/飞机.pcd")
# ----------------------------定义x、y散点坐标---------------------------------
points = np.asarray(pcd.points)
x = points[:, 0]
y = points[:, 1]

# 绘图
plot1 = plt.plot(x, y, 'ms', label='orig1')

popt, pcov = curve_fit(func, x, y)
yy2 = [func(i, popt[0], popt[1], popt[2], popt[3], popt[4], popt[5], popt[6], popt[7]) for i in x]
plt.plot(x, yy2, 'k-', label='fit')

print(u'系数a:', popt[0])
print(u'系数b:', popt[1])
print(u'系数c:', popt[2])
print(u'系数d:', popt[3])
print(u'系数e:', popt[4])
print(u'系数f:', popt[5])
print(u'系数g:', popt[6])
print(u'系数h:', popt[7])
plt.xlabel('x')
plt.ylabel('y')
plt.legend(loc=2, bbox_to_anchor=(1.05, 1.0), borderaxespad=0.)
plt.title(u'Curve Poly Fitting')
plt.subplots_adjust(right=0.75)
plt.show()

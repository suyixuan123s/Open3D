"""
Author: Yixuan Su
Date: 2025/02/16 17:58
File: 间接平差拟合二维直线.py
Description: 

"""

import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt


def line_fit_2d(pts):
    """
    间接平差法拟合二维直线
    :param pts: 输入点云
    :return: 直线y=kx+b参数：k,b
    """
    n = pts.shape[0]  # 点的个数
    B = np.ones((n, 2))  #
    l = np.ones((n, 1))  #
    # 构建计算所需矩阵
    B[:, 0] = pts[:, 0]
    l[:, 0] = pts[:, 1]

    NBB = np.dot(B.T, B)
    NBB_inv = np.linalg.inv(NBB)
    W = np.dot(B.T, l)
    X = NBB_inv.dot(W)

    return X


# --------------------------加载点云--------------------------------
pcd = o3d.io.read_point_cloud('../../../data/1.pcd')
points = np.asarray(pcd.points)
# -------------------------拟合二维直线-----------------------------
line_parm = line_fit_2d(points)
print("k=", line_parm[0])
print("b=", line_parm[1])
# ------------------------可视化拟合结果----------------------------
x = points[:, 0]
y = points[:, 1]
m = line_parm[0]
c = line_parm[1]
plt.plot(x, y, 'o', label='Original data', markersize=3)
plt.plot(x, m * x + c, 'r', label='Fitted line')
plt.title("Fit 2d line by indirect adjustment method")
plt.legend()
plt.show()

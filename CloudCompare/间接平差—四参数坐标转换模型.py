"""
Author: Yixuan Su
Date: 2025/02/15 20:29
File: 间接平差—四参数坐标转换模型.py
Description: 

"""
import numpy as np


def four_parameter_coordinate_conversion(old_pts, new_pts):
    """
    四参数坐标转换
    :param old_pts: 旧坐标系
    :param new_pts: 新坐标系
    :return: 四参数计算结果
    """
    if old_pts.shape[0] != new_pts.shape[0]:
        return False
    if old_pts.shape[0] < 3:
        return False
    n = old_pts.shape[0]  # 点的个数
    B = np.zeros((n * 2, 4))  # 矩阵B
    L = np.zeros((n * 2, 1))  # 矩阵L
    X = np.ones((4, 1))  # 解X
    deltX = np.ones((4, 1))  # 迭代增量deltX
    # 四参数的初始值
    x0 = 0.0
    y0 = 0.0
    lamda = 1.0
    alpha = 0.0
    X[0] = x0
    X[1] = y0
    X[2] = lamda * np.cos(alpha)
    X[3] = lamda * np.sin(alpha)
    # 构建计算所需矩阵
    for i in range(0, n):
        B[2 * i, 0] = 1
        B[2 * i, 1] = 0
        B[2 * i, 0] = old_pts[i].x
        B[2 * i, 1] = -old_pts[i].y
        B[2 * i + 1, 0] = 0
        B[2 * i + 1, 1] = 1
        B[2 * i + 1, 0] = old_pts[i].y
        B[2 * i + 1, 1] = old_pts[i].x

        L[2 * i] = new_pts[i].x
        L[2 * i + 1] = new_pts[i].y
    # 迭代计算
    count = 0
    while np.linalg.norm(deltX) > 0.0001:
        l = L - B.dot(X)
        NBB = B.T.dot(B)
        W = B.T.dot(l)
        deltX = np.linalg.inv(NBB).dot(W)

        X += deltX
        count += 1
        if count >= 10:
            break
    print("迭代次数:", count)

    return X

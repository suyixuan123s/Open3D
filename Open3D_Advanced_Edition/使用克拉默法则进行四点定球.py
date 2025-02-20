"""
Author: Yixuan Su
Date: 2025/02/16 16:52
File: 使用克拉默法则进行四点定球.py
Description: 

"""
import numpy as np


def four_point_fix_sphere(points):
    # 四个点的坐标
    P1 = points[0]
    P2 = points[1]
    P3 = points[2]
    P4 = points[3]
    # 互相相减
    l = P1 - P2
    l1 = P3 - P4
    l2 = P2 - P3
    # 系数D
    D = np.ones((3, 3))
    D[0, :] = l
    D[1, :] = l1
    D[2, :] = l2
    detD = np.linalg.det(D)
    # P,Q,R
    P = (np.linalg.norm(P1) ** 2 - np.linalg.norm(P2) ** 2) * 0.5
    Q = (np.linalg.norm(P3) ** 2 - np.linalg.norm(P4) ** 2) * 0.5
    R = (np.linalg.norm(P2) ** 2 - np.linalg.norm(P3) ** 2) * 0.5
    # 常数项系数
    L = np.ones((3, 1))
    L[0] = P
    L[1] = Q
    L[2] = R
    # 系数Dx
    Dx = np.ones((3, 3))
    Dx[:, :] = D[:, :]
    Dx[:, 0] = L[:, 0]
    detDx = np.linalg.det(Dx)

    # 系数Dy
    Dy = np.ones((3, 3))
    Dy[:, :] = D[:, :]
    Dy[:, 1] = L[:, 0]
    detDy = np.linalg.det(Dy)
    # 系数Dz
    Dz = np.ones((3, 3))
    Dz[:, :] = D[:, :]
    Dz[:, 2] = L[:, 0]
    detDz = np.linalg.det(Dz)
    # 计算球心
    xo = detDx / detD
    yo = detDy / detD
    zo = detDz / detD
    sphere_o = [xo, yo, zo]
    # 计算半径
    sphere_r = np.linalg.norm(P1 - sphere_o)

    return sphere_o, sphere_r

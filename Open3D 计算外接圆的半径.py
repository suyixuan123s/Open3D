"""
Author: Yixuan Su
Date: 2025/02/19 20:50
File: Open3D 计算外接圆的半径.py
Description: 

"""
import numpy as np


def get_circum_circle_radius(point1, point2, point3):
    """
    已知空间内的三个点，计算外接圆半径
    :param point1: 点1
    :param point2: 点2
    :param point3: 点3
    :return: 外接圆半径
    """
    a = np.linalg.norm(point3 - point1)
    b = np.linalg.norm(point3 - point2)
    c = np.linalg.norm(point1 - point2)
    p = (a + b + c) / 2
    s = np.sqrt(p * (p - a) * (p - b) * (p - c))
    radius = a * b * c / (4 * s)

    return radius


A = np.array([19, 22, 19])
B = np.array([-89, 43, -78])
C = np.array([45, 12, 21])
r = get_circum_circle_radius(A, B, C)
print('外接圆半径为：', r)

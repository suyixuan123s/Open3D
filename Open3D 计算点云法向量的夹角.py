"""
Author: Yixuan Su
Date: 2025/02/19 22:26
File: Open3D 计算点云法向量的夹角.py
Description: 

"""
import numpy as np


def get_angle_vector(a, b, degrees=True):
    """"
    计算法向量的夹角，
    a,b：输入的法向量
    degrees：True输出为角度制，False输出为弧度制。
    """
    cos_a = np.inner(a, b) / (np.linalg.norm(a) * np.linalg.norm(b))
    if cos_a < -1:
        cos_a = np.array([-1])
    if cos_a > 1:
        cos_a = np.array([1])
    rad = np.arccos(cos_a)  # 计算结果为弧度制
    deg = np.rad2deg(rad)  # 转化为角度制
    angle = deg if degrees else rad
    return angle


x0 = np.array([2, 4, 7])
x1 = np.array([7, 8, 9])
print(get_angle_vector(x0, x1))

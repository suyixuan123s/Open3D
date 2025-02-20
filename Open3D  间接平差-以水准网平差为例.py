"""
Author: Yixuan Su
Date: 2025/02/18 10:20
File: Open3D  间接平差-以水准网平差为例.py
Description: 

"""
import numpy as np


def indirect_adjustment(n, t, B, l, P):
    """
    间接平差——以水准网平差为例
    :param n: 观测值个数
    :param t: 参数个数
    :param B: 误差方程系数矩阵数组长度为 n*t，已知
    :param l: 观测值向量，数组长度为n，已知
    :param P: 观测值权数组，只有权矩阵的对角线元素数组长度为n，已知
    :return: x:法方程的解，V:改正数，μ单位权中误差
    """

    NBB = B.T.dot(P).dot(B)  # NBB=B^TPB
    W = B.T.dot(P).dot(l)  # W=B^TPl
    NBB_inv = np.linalg.inv(NBB)  # NBB^-1
    x = NBB_inv.dot(W)  # 参数平差值向量，数组长度为t，待计算 x=NBB^-1W
    V = B.dot(x) - l  # 观测值残差向量，数组长度为n，待计算 v=Bx-l
    μ = np.sqrt(V.T.dot(P).dot(V) / (n - t))  # 单位权中误差μ

    return x, V, μ


if __name__ == "__main__":
    # 观测值总数和参数总数
    n = 5  # 观测值总数
    t = 3  # 参数总数
    # 观测高差
    L = np.asarray([5.835, 3.782, 9.640, 7.384, 2.270])
    # A点高程
    HA = np.asarray([237.483])
    # 计算参数的近似值
    X10 = HA + L[0]
    X20 = HA + L[2]
    X30 = HA + L[4]
    X0 = np.hstack([X10, X20, X30])
    # 误差方程系数
    B = np.asarray(
        [[1, 0, 0],
         [-1, 1, 0],
         [0, 1, 0],
         [0, 1, -1],
         [0, 0, 1]], dtype=np.float64)
    l = np.asarray([0, -23, 0, 14, 0], dtype=np.float64).T
    # 定权，得观测值的权阵
    P = np.diag([2.9, 3.7, 2.5, 3.3, 4.0])
    x, V, μ = indirect_adjustment(n, t, B, l, P)
    print("法方程的解x:\n", x)  # 单位mm
    print("改正数V:\n", V)
    print("参数平差值:\n", X0 + x / 1000)  # X0的单位为m，需要进行统一。
    print("观测值的平差值:\n", L + V)
    print("单位权中误差: ", μ)

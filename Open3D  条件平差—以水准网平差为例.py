"""
Author: Yixuan Su
Date: 2025/02/18 10:25
File: Open3D  条件平差—以水准网平差为例.py
Description: 

"""
import numpy as np


def condition_adjustment(n, t, A, W, Q):
    """
    条件平差——以水准网平差为例
    :param n: 观测值个数
    :param t: 参数个数
    :param A: 误差方程系数矩阵,数组长度为 n*(n-t)，已知
    :param W: 观测值向量,数组长度为n-t，已知
    :param Q: 协因数阵,元素数组长度为nxn，已知
    :return: K:法方程的解，V:改正数，μ单位权中误差
    """

    NAA = A.dot(Q).dot(A.T)  # NAA=AQA'
    NAA_inv = np.linalg.inv(NAA)  # NAA^-1
    K = -NAA_inv.dot(W)  # 参数平差值,待计算 K= -NAA^-1W
    V = Q.dot(A.T).dot(K)  # V = QA'K
    μ = np.sqrt(V.T.dot(np.linalg.inv(Q)).dot(V) / (n - t))  # 单位权中误差μ

    return K, V, μ


if __name__ == "__main__":
    # 观测值总数和参数总数
    n = 7  # 观测值总数
    t = 3  # 参数总数
    # 由条件方程知系数阵为
    A = np.asarray(
        [[1, -1, 0, 0, 1, 0, 0],
         [0, 0, 1, -1, 1, 0, 0],
         [0, 0, 1, 0, 0, 1, 1],
         [0, 1, 0, -1, 0, 0, 0]], dtype=np.float64)

    W = np.asarray([7, 8, 6, -3], dtype=np.float64).T
    Q = np.diag([1.1, 1.7, 2.3, 2.7, 2.4, 1.4, 2.6])
    K, V, μ = condition_adjustment(n, t, A, W, Q)
    print("法方程的解K:\n", K)
    print("改正数V:\n", V)
    print("单位权中误差: ", μ)

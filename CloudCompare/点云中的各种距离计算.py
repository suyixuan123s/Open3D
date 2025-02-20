"""
Author: Yixuan Su
Date: 2025/02/15 17:56
File: 点云中的各种距离计算.py
Description: 

"""
import numpy as np


def euclidean(x, y):
    # 欧氏距离
    el = np.sqrt(np.sum((x - y) ** 2))
    return el


def manhattan(x, y):
    # 曼哈顿距离
    mh = np.sum(np.abs(x - y))
    return mh


def chebyshev(x, y):
    # 切比雪夫距离
    cs = np.max(np.abs(x - y))
    return cs


def minkowski(x, y, p):
    # 闵可夫斯基距离
    ms = np.sum(np.abs(x - y) ** p) ** (1 / p)
    return ms


def hamming(x, y):
    # 汉明距离
    hm = np.sum(x != y) / len(x)
    return hm


if __name__ == '__main__':
    a = np.array([1, 2, 3])
    b = np.array([4, 5, 6])
    print('a,b之间的欧氏距离为：', euclidean(a, b))


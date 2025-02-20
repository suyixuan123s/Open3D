"""
Author: Yixuan Su
Date: 2025/02/19 23:19
File: Open3D 对应点集配准的四元数法.py
Description: 

"""
import open3d as o3d
import numpy as np


def solve_transform_quaternion(s, t):
    P = np.asarray(s.points)
    Q = np.asarray(t.points)
    # 判断两个点集中点的个数是否一致
    if P.shape[0] != Q.shape[0]:
        raise Exception("两个点集不匹配")
    else:
        n = P.shape[0]
    # 1、分别求质心
    meanP = np.mean(P, axis=0)
    meanQ = np.mean(Q, axis=0)
    # 2、去质心
    P_ = P - meanP
    Q_ = Q - meanQ
    # 3、构建协方差矩阵D
    D = np.dot(P_.T, Q_) / n
    # 4、计算4 X 4 矩阵中所需元素
    Dt = np.transpose(D)  # D的转置
    tr_D = np.trace(D)  # D的迹
    A_ij = D - Dt
    v_A = np.array([[A_ij[1][2], A_ij[2][0], A_ij[0][1]]])
    M = D + Dt - tr_D * np.eye(3)
    # 5、构建 4 X 4 矩阵
    Ql = np.vstack((np.array([[tr_D]]), np.transpose(v_A)))
    Qr = np.vstack((v_A, M))
    Q = np.hstack((Ql, Qr))
    # 6、求 4 X4 矩阵的特征值与特征向量
    [eig_v, eig_u] = np.linalg.eig(Q)
    i = np.argmax(eig_v)  # 最大特征值的位置
    q = eig_u[:, i]  # 最大特征值对应的特征向量
    # 7、由四元数求旋转矩阵
    R = o3d.geometry.get_rotation_matrix_from_quaternion(q)
    t = meanQ - np.dot(R, meanP)  # 计算平移向量
    # 8、构建欧式变换矩阵
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    T[3, 3] = 1.0
    return T


# ---------------加载点云数据--------------------
source = o3d.io.read_point_cloud("../data/1.pcd")
target = o3d.io.read_point_cloud("../data/2.pcd")
# ------对初始位置的点云进行颜色渲染--------------
source.paint_uniform_color([0, 1, 0])  # 绿色
target.paint_uniform_color([0, 0, 1])  # 蓝色
o3d.visualization.draw_geometries([source, target], width=800, height=800)

Tran = solve_transform_quaternion(source, target)
print('变换矩阵为:\n', Tran)
align = source.transform(Tran)
align.paint_uniform_color([1, 0, 0])  # 红色
o3d.visualization.draw_geometries([align, target], width=800, height=800)

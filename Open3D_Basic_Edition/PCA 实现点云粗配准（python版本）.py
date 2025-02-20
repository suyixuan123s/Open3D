"""
Author: Yixuan Su
Date: 2025/02/11 16:21
File: PCA 实现点云粗配准（python版本）.py
Description: 

"""
import open3d as o3d
import numpy as np
import copy


# SVD分解计算点云的质心与特征向量
def pca_compute(data):
    [center, covariance] = data.compute_mean_and_covariance()
    # SVD奇异值分解，得到covariance矩阵的特征值和特征向量
    eigenvectors, _, _ = np.linalg.svd(covariance)
    return eigenvectors, center


# PCA实现点云配准
def pca_registration(P, X):

    P.paint_uniform_color([1, 0, 0])  # 给原始点云赋色
    X.paint_uniform_color([0, 1, 0])

    error = []   # 定义误差集合
    matrax = []  # 定义变换矩阵集合
    Up, Cp = pca_compute(P)  # PCA方法得到P对应的特征向量、点云质心
    Ux, Cx = pca_compute(X)  # PCA方法得到X对应的特征向量、点云质心
    # 主轴对应可能出现的情况
    Upcopy = Up
    sign1 = [1, -1, 1, 1, -1, -1, 1, -1]
    sign2 = [1, 1, -1, 1, -1, 1, -1, -1]
    sign3 = [1, 1, 1, -1, 1, -1, -1, -1]
    for nn in range(len(sign3)):
        Up[0] = sign1[nn]*Upcopy[0]
        Up[1] = sign2[nn]*Upcopy[1]
        Up[2] = sign3[nn]*Upcopy[2]
        R0 = np.dot(Ux, np.linalg.inv(Up))
        T0 = Cx-np.dot(R0, Cp)
        T = np.eye(4)
        T[:3, :3] = R0
        T[:3, 3] = T0
        T[3, 3] = 1
# 计算配准误差，误差最小时对应的变换矩阵即为最终完成配准的变换
        trans = copy.deepcopy(P).transform(T)
        dists = trans.compute_point_cloud_distance(X)
        dists = np.asarray(dists)  # 欧氏距离（单位是：米）
        mse = np.average(dists)
        error.append(mse)
        matrax.append(T)
    ind = error.index(min(error))  # 获取误差最小时对应的索引
    final_T = matrax[ind]  # 根据索引获取变换矩阵
    print("变换矩阵为：\n", final_T)
    pcaregisted = copy.deepcopy(P).transform(final_T)
    return pcaregisted  # 返回配准后的点云


if __name__ == "__main__":
    source = o3d.io.read_point_cloud("../../data/1.pcd")
    target = o3d.io.read_point_cloud("../../data/2.pcd")
    final = pca_registration(source, target)

    o3d.visualization.draw_geometries([source, target],
                                      window_name="原始点云",
                                      width=1024, height=768,
                                      left=50, top=50,
                                      mesh_show_back_face=False)
    o3d.visualization.draw_geometries([final, target],
                                      window_name="PCA配准后的点云",
                                      width=1024, height=768,
                                      left=50, top=50,
                                      mesh_show_back_face=False)


"""
Author: Yixuan Su
Date: 2025/02/16 17:43
File: 方向向量约束的PCA快速粗配准.py
Description: 

"""
import numpy as np
import open3d as o3d
from sklearn.decomposition import PCA


def calculate_rotationMatrix_from_vectors(u, v):
    """
    根据向量计算旋转矩阵
    :param u: 向量u
    :param v: 向量v
    :return: 旋转矩阵
    """
    w = np.cross(u, v)
    c = np.dot(u, v)
    s = np.linalg.norm(w)

    Sx = np.asarray([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])
    R = np.eye(3) + Sx + Sx.dot(Sx) * ((1 - c) / (s ** 2))
    return R


# ----------------------------加载点云数据---------------------------------
source = o3d.io.read_point_cloud('../data/1.pcd')
target = o3d.io.read_point_cloud('../data/2.pcd')
src = np.asarray(source.points)
tgt = np.asarray(target.points)
# -----------------------------调用PCA------------------------------------
pcas = PCA(n_components=3)  # 设置保留主成分个数
pcas.fit(src)
pcat = PCA(n_components=3)  # 设置保留主成分个数
pcat.fit(tgt)
# ----------------------------计算特征向量---------------------------------
sVector = pcas.components_  # 按行排列，第一主成分排在首行
tVector = pcat.components_  # 按行排列，第一主成分排在首行
# ------------------------------计算质心----------------------------------
scentrid = pcas.mean_  # 源点云质心
tcentrid = pcat.mean_  # 目标点云质心
# 使用该方向判别条件，基于第一主成分进行配准可省去大量的迭代计算过程
if np.dot(sVector[0, :], tVector[0, :]) < 0:
    tVector[0, :] *= -1
R = calculate_rotationMatrix_from_vectors(sVector[0, :], tVector[0, :])
T = np.eye(4)
T0 = tcentrid - np.dot(R, scentrid)  # 计算平移
T[:3, :3] = R
T[:3, 3] = T0
final = source.transform(T)
# ----------------------------结果可视化---------------------------------
o3d.visualization.draw_geometries([final, target],
                                  window_name="PCA配准后的点云",
                                  width=1024, height=768,
                                  left=50, top=50,
                                  mesh_show_back_face=False)

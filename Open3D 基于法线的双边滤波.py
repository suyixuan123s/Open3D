"""
Author: Yixuan Su
Date: 2025/02/18 16:07
File: Open3D 基于法线的双边滤波.py
Description: 

"""
import open3d as o3d
import numpy as np

# ----------------------------------加载点云------------------------------------
pcd = o3d.io.read_point_cloud("../data/JZDM.pcd")
o3d.visualization.draw_geometries([pcd])
# ------------------------------基于法线的双边滤波--------------------------------
# 参数设置
sigma_s = 0.05  # 距离权重
sigma_r = 10  # 法向权重
searchNum = 30  # 近邻点数
# 计算法向量
pcd.estimate_normals(o3d.geometry.KDTreeSearchParamKNN(searchNum))
# 构建KD-tree
kdtree = o3d.geometry.KDTreeFlann(pcd)
BFilter = o3d.geometry.PointCloud(pcd)
# 双边滤波
for i in range(len(pcd.points)):
    [k, idx, _] = kdtree.search_knn_vector_3d(pcd.points[i], searchNum)
    curNormal = pcd.normals[i]
    searchPoint = pcd.points[i]

    BF = 0.0
    W = 0.0
    for j in idx[1:]:
        near_point = pcd.points[j]
        vec = near_point - pcd.points[i]
        dd = np.sqrt(np.sum(np.square(vec)))
        dn = np.dot(vec, curNormal)
        weight = np.exp(-dd * dd / (2 * sigma_s * sigma_s)) * np.exp(-dn * dn / (2 * sigma_r * sigma_r))
        BF = BF + weight * dn
        W = W + weight

    lamda = BF / W
    BFilter.points[i] = searchPoint + lamda * curNormal
# ---------------------------------结果可视化----------------------------------
o3d.visualization.draw_geometries([BFilter], window_name="基于法线的双边滤波",
                                  width=1024, height=768,
                                  left=50, top=50,
                                  mesh_show_back_face=False)

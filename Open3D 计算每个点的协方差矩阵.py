"""
Author: Yixuan Su
Date: 2025/02/18 15:23
File: Open3D 计算每个点的协方差矩阵.py
Description: 

"""
import open3d as o3d


# ---------------------------------加载点云----------------------------------
pcd = o3d.io.read_point_cloud("../data/飞机.pcd")
# -----------------------------计算每个点的协方差矩阵-------------------------
pcd.estimate_covariances(o3d.geometry.KDTreeSearchParamKNN(30))
cov_mat = pcd.covariances     # 获取每个点的协方差矩阵
print("协方差矩阵计算完毕！！！")
print("第0个点的协方差矩阵为：\n", cov_mat[0])


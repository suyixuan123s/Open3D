"""
Author: Yixuan Su
Date: 2025/02/18 15:12
File: Open3D 计算点云粗糙度（方法二）.py
Description: 

"""

import open3d as o3d
import numpy as np
from matplotlib import pyplot as plt

# ---------------------------------加载点云----------------------------------
pcd = o3d.io.read_point_cloud("../data/飞机.pcd")
# -----------------------------计算每个点的协方差矩阵-------------------------
pcd.estimate_covariances(o3d.geometry.KDTreeSearchParamKNN(30))
cov_mat = pcd.covariances  # 获取每个点的协方差矩阵
num_of_pts = len(pcd.points)  # 点云点的个数
roughness = np.zeros(num_of_pts)  # 初始化存储每个点粗糙度的容器
# ------------------------------计算每个点的粗糙度---------------------------
for i in range(num_of_pts):
    eignvalue, _ = np.linalg.eig(cov_mat[i])  # 计算特征值
    if np.any(np.isnan(eignvalue)) or np.any(eignvalue < 0):
        roughness[i] = 0  # 如果存在 NaN 或负特征值，粗糙度为0
    else:
        idx = eignvalue.argsort()[::-1]
        eignvalue = eignvalue[idx]
        roughness[i] = np.sqrt(eignvalue[2])  # 计算粗糙度


# for i in range(num_of_pts):
#     eignvalue, _ = np.linalg.eig(cov_mat[i])  # SVD分解求特征值
#     idx = eignvalue.argsort()[::-1]
#     eignvalue = eignvalue[idx]
#     roughness[i] = np.sqrt(eignvalue[2])

# ---------------------------使用伪颜色对点云进行渲染-------------------------
zhot_colors = plt.get_cmap('hot')(  # hot表示为热力图
    (roughness - roughness.min()) / (roughness.max() - roughness.min()))
zhot_colors = zhot_colors[:, :3]
pcd.colors = o3d.utility.Vector3dVector(zhot_colors)

o3d.io.write_point_cloud("render.pcd", pcd)
o3d.visualization.draw_geometries([pcd], window_name="点云热力图渲染赋色粗糙度",
                                  width=1024, height=768,
                                  left=50, top=50,
                                  mesh_show_back_face=False)

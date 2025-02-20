"""
Author: Yixuan Su
Date: 2025/02/19 19:55
File: Open3D 大场景点云水平面校准.py
Description: 

"""
import numpy as np
import open3d as o3d
import pyransac3d as pyrsc

# -------------------读取点云数据并可视化------------------------
pcd_load = o3d.io.read_point_cloud("../data/tree2.pcd")
o3d.visualization.draw_geometries([pcd_load])
points = np.asarray(pcd_load.points)
# --------------------RANSAC平面拟合----------------------------
plane_model, inliers = pcd_load.segment_plane(distance_threshold=0.3,
                                              ransac_n=10,
                                              num_iterations=100)
normal = plane_model[0:3]
print("地面的法向量为:\n", normal)
# ------------------根据向量创建旋转矩阵-------------------------
R = pyrsc.get_rotationMatrix_from_vectors(normal, [0, 0, 1])
T = np.eye(4)
T[:3, :3] = R
print('地面校准的变换矩阵为：\n', T)
pcd_r = pcd_load.transform(T)
o3d.io.write_point_cloud("cal_cloud1.pcd", pcd_r)


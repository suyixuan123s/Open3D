"""
Author: Yixuan Su
Date: 2025/02/18 10:17
File: Open3D  删除点云中重复的点.py
Description: 

"""
import open3d as o3d


# --------------------------------------加载点云-------------------------------------
dataset = o3d.data.PCDPointCloud()
pcd = o3d.io.read_point_cloud("../data/bgm_cluster1.pcd")
print(pcd)  # 输出点云点的个数
o3d.visualization.draw_geometries([pcd])

pcd.remove_duplicated_points()
print(pcd)  # 输出点云点的个数
# ----------------------------------结果可视化---------------------------------------
o3d.visualization.draw_geometries([pcd],
                                  zoom=0.62,
                                  front=[0.4361, -0.2632, -0.8605],
                                  lookat=[2.4947, 1.7728, 1.5541],
                                  up=[-0.1726, -0.9630, 0.2071])



"""
Author: Yixuan Su
Date: 2025/02/18 20:11
File: Open3D Ransac拟合分割多个球体.py
Description: 

"""
import open3d as o3d
import numpy as np
import pyransac3d as pyrsc

# ------------------------------------读取点云---------------------------------------
pcd = o3d.io.read_point_cloud("../data/Sphere.pcd")
# ------------------------------------参数设置---------------------------------------
segment = []  # 存储分割结果的容器
min_num = 15  # 每个分割球体所需的最小点数
dist = 0.1  # Ransac分割的距离阈值
iters = 0  # 用于统计迭代次数，非待设置参数
# -----------------------------------分割多个球体-------------------------------------
while len(pcd.points) > min_num:

    points = np.asarray(pcd.points)
    sph = pyrsc.Sphere()
    center, radius, inliers = sph.fit(points, thresh=dist, maxIteration=100)

    sphere_cloud = pcd.select_by_index(inliers)  # 分割出的球体点云
    r_color = np.random.uniform(0, 1, (1, 3))  # 球体点云随机赋色
    sphere_cloud.paint_uniform_color([r_color[:, 0], r_color[:, 1], r_color[:, 2]])
    pcd = pcd.select_by_index(inliers, invert=True)  # 剩余的点云
    segment.append(sphere_cloud)
    file_name = "RansacFitMutiSphere" + str(iters + 1) + ".pcd"
    o3d.io.write_point_cloud(file_name, sphere_cloud)
    iters += 1
    if len(inliers) < min_num:
        break
# ------------------------------------结果可视化--------------------------------------
o3d.visualization.draw_geometries(segment, window_name="Ransac分割多个球体",
                                  width=1024, height=768,
                                  left=50, top=50,
                                  mesh_show_back_face=False)

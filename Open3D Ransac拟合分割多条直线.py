"""
Author: Yixuan Su
Date: 2025/02/18 20:40
File: Open3D Ransac拟合分割多条直线.py
Description: 

# 计算机是的视觉
"""

import open3d as o3d
import numpy as np
import pyransac3d as pyrsc

# ------------------------------------读取点云---------------------------------------
pcd = o3d.io.read_point_cloud("../data/L.pcd")
o3d.visualization.draw_geometries([pcd])
# ------------------------------------参数设置---------------------------------------
segment = []  # 存储分割结果的容器
min_num = 15  # 每个分割直线所需的最小点数
dist = 0.1  # Ransac分割的距离阈值
iters = 0  # 用于统计迭代次数，非待设置参数
# -----------------------------------分割多个直线-------------------------------------
while len(pcd.points) > min_num:

    points = np.asarray(pcd.points)
    line = pyrsc.Line()
    A, B, inliers = line.fit(points, thresh=dist, maxIteration=100)

    line_cloud = pcd.select_by_index(inliers)  # 分割出的直线点云
    r_color = np.random.uniform(0, 1, (1, 3))  # 直线点云随机赋色
    line_cloud.paint_uniform_color([r_color[:, 0], r_color[:, 1], r_color[:, 2]])
    pcd = pcd.select_by_index(inliers, invert=True)  # 剩余的点云
    segment.append(line_cloud)
    file_name = "RansacFitMutiLine" + str(iters + 1) + ".pcd"
    o3d.io.write_point_cloud(file_name, line_cloud)
    iters += 1
    if len(inliers) < min_num:
        break
# ------------------------------------结果可视化--------------------------------------
o3d.visualization.draw_geometries(segment, window_name="Ransac分割多个直线",
                                  width=1024, height=768,
                                  left=50, top=50,
                                  mesh_show_back_face=False)

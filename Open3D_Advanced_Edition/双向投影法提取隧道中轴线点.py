"""
Author: Yixuan Su
Date: 2025/02/16 16:35
File: 双向投影法提取隧道中轴线点.py
Description: 

"""
import open3d as o3d
import numpy as np

# -----------------------------读取点云------------------------------------
cloud = o3d.io.read_point_cloud("9targetGridCloud.pcd")
o3d.visualization.draw_geometries([cloud], window_name="隧道点云")
step = 0.1  # 格网大小

# -----------------------------创建格网------------------------------------
point_cloud = np.asarray(cloud.points)

# 1、获取点云数据边界
x_min, y_min, z_min = np.amin(point_cloud, axis=0)
x_max, y_max, z_max = np.amax(point_cloud, axis=0)

# 2、计算格网行列数
rowNum = np.ceil((y_max - y_min) / step)

h = list()  # h 为保存索引的列表

# 3、获取每个点对应的格网
for i in range(len(point_cloud)):
    rowID = np.floor((point_cloud[i][1] - y_min) / step)
    h.append(rowID)
h = np.array(h)

# 4、提取中轴线
middle_line = []
h_indice = np.argsort(h)  # 返回h里面的元素按从小到大排序的索引
h_sorted = h[h_indice]
begin = 0
for i in range(len(h_sorted) - 1):
    if h_sorted[i] == h_sorted[i + 1]:
        continue
    else:
        point_idx = h_indice[begin: i + 1]
        pts_grid = point_cloud[point_idx]
        minX, minY, minZ = np.amin(pts_grid, axis=0)
        maxX, maxY, maxZ = np.amax(pts_grid, axis=0)
        midX = (minX + maxX) * 0.5
        midY = (minY + maxY) * 0.5
        midZ = (minZ + maxZ) * 0.5
        middle_line.append([midX, midY, midZ])

    begin = i + 1

middle_line = np.array(middle_line, dtype=np.float64)
ml_cloud = o3d.geometry.PointCloud()

ml_cloud.points = o3d.utility.Vector3dVector(middle_line)
o3d.io.write_point_cloud("middle_line_cloud.pcd", ml_cloud)
o3d.visualization.draw_geometries([cloud, ml_cloud])

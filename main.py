"""
Author: Yixuan Su
Date: 2025/02/18 16:05
File: main.py
Description: 

"""
import open3d as o3d
import numpy as np
import regiongrowing as reg

# ------------------------------读取点云---------------------------------------
pcd = o3d.io.read_point_cloud("../data/JZDM.pcd")
o3d.visualization.draw_geometries([pcd], window_name="原始点云",
                                  width=1024, height=768,
                                  left=50, top=50,
                                  mesh_show_back_face=False)
# ------------------------------区域生长---------------------------------------
rg = reg.RegionGrowing(pcd,
                       min_pts_per_cluster=500,  # 每个聚类的最小点数
                       max_pts_per_cluster=100000,  # 每个聚类的最小点数
                       neighbour_number=30,  # 邻域搜索点数
                       theta_threshold=30,  # 平滑阈值（角度制）
                       curvature_threshold=0.05)  # 曲率阈值
# ---------------------------聚类结果分类保存----------------------------------
indices = rg.extract()
print("聚类个数为", len(indices))
segment = []  # 存储分割结果的容器
for i in range(len(indices)):
    ind = indices[i]
    clusters_cloud = pcd.select_by_index(ind)
    r_color = np.random.uniform(0, 1, (1, 3))  # 分类点云随机赋色
    clusters_cloud.paint_uniform_color([r_color[:, 0], r_color[:, 1], r_color[:, 2]])
    segment.append(clusters_cloud)
    # 保存到本地文件夹
    # file_name = "Region_growing_cluster" + str(i + 1) + ".pcd"
    # o3d.io.write_point_cloud(file_name, clusters_cloud)
# -----------------------------结果可视化------------------------------------
o3d.visualization.draw_geometries(segment, window_name="区域生长分割",
                                  width=1024, height=768,
                                  left=50, top=50,
                                  mesh_show_back_face=False)

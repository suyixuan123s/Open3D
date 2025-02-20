"""
Author: Yixuan Su
Date: 2025/02/16 17:47
File: DBSCAN点云聚类.py
Description: 

"""
import os

import open3d as o3d
from sklearn.preprocessing import StandardScaler
import numpy as np
from sklearn.cluster import DBSCAN

# ----------------------------加载点云数据---------------------------------
pcd = o3d.io.read_point_cloud("../data/two_tree.pcd")
X = np.asarray(pcd.points)

X = StandardScaler().fit_transform(X)
# ------------------------------密度聚类-----------------------------------
db = DBSCAN(eps=0.2, min_samples=10).fit(X)
labels = db.labels_
# Number of clusters in labels, ignoring noise if present.
n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)
n_noise_ = list(labels).count(-1)
print("聚类个数为 : %d" % n_clusters_)
# ------------------------使用open3d获取点云分类结果------------------------
segment = []  # 存储分割结果的容器
pcd_folder_path = "data/DBSCAN点云聚类/"

if not os.path.exists(pcd_folder_path):
    os.makedirs(pcd_folder_path)

for i in range(n_clusters_):
    ind = np.where(labels == i)[0]
    clusters_cloud = pcd.select_by_index(ind)
    r_color = np.random.uniform(0, 1, (1, 3))
    clusters_cloud.paint_uniform_color([r_color[:, 0], r_color[:, 1], r_color[:, 2]])
    segment.append(clusters_cloud)
    file_name = pcd_folder_path + "dbscan_cluster" + str(i + 1) + ".pcd"
    o3d.io.write_point_cloud(file_name, clusters_cloud)
# -------------------------------结果可视化--------------------------------
o3d.visualization.draw_geometries(segment, window_name="DBSCAN点云聚类",
                                  width=1024, height=768,
                                  left=50, top=50,
                                  mesh_show_back_face=False)

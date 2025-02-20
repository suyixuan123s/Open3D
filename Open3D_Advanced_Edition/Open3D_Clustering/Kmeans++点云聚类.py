"""
Author: Yixuan Su
Date: 2025/02/16 17:51
File: Kmeans++点云聚类.py
Description: 

"""
import os
import time
import numpy as np
import open3d as o3d
from sklearn.cluster import KMeans

# ----------------------------加载点云数据---------------------------------
pcd = o3d.io.read_point_cloud("data/two_tree.pcd")
X = np.asarray(pcd.points)
# ---------------------------Kmeans++聚类----------------------------------
k_means = KMeans(init="k-means++", n_clusters=2, n_init=10)
t0 = time.time()
k_means.fit(X)
labels = k_means.predict(X)
t_batch = time.time() - t0
labels_unique = np.unique(labels)
n_clusters_ = len(labels_unique)
print("聚类个数为 : %d" % n_clusters_)
print("聚类用时： %.3f sec.\n" % t_batch)
# ------------------------使用open3d获取点云分类结果----------------------
segment = []  # 存储分割结果的容器
pcd_folder_path = "data/Kmeans++点云聚类/"

if not os.path.exists(pcd_folder_path):
    os.makedirs(pcd_folder_path)

for i in range(n_clusters_):
    ind = np.where(labels == i)[0]
    clusters_cloud = pcd.select_by_index(ind)
    r_color = np.random.uniform(0, 1, (1, 3))
    clusters_cloud.paint_uniform_color([r_color[:, 0], r_color[:, 1], r_color[:, 2]])
    segment.append(clusters_cloud)
    file_name = pcd_folder_path + "kmeans++_cluster" + str(i + 1) + ".pcd"
    o3d.io.write_point_cloud(file_name, clusters_cloud)
# -----------------------------结果可视化-------------------------------
o3d.visualization.draw_geometries(segment, window_name="Kmeans++点云聚类",
                                  width=1024, height=768,
                                  left=50, top=50,
                                  mesh_show_back_face=False)

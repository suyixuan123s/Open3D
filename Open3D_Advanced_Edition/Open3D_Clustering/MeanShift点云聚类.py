"""
Author: Yixuan Su
Date: 2025/02/16 17:16
File: MeanShift点云聚类.py
Description: 

"""
import open3d as o3d
import numpy as np
from sklearn.cluster import MeanShift, estimate_bandwidth

# ----------------------------加载点云数据---------------------------------
pcd = o3d.io.read_point_cloud("../../../data/two_tree.pcd")
X = np.asarray(pcd.points)
# -----------------------使用MeanShift算法进行聚类-------------------------
# estimate_bandwidth函数可以自动获取 一个bandwidth估算值，也可自行设置bandwidth
bandwidth = estimate_bandwidth(X, quantile=0.2, n_samples=500)
ms = MeanShift(bandwidth=bandwidth, bin_seeding=True)
ms.fit(X)
labels = ms.labels_
cluster_centers = ms.cluster_centers_
labels_unique = np.unique(labels)
n_clusters_ = len(labels_unique)
print("聚类个数为 : %d" % n_clusters_)
# ------------------------使用open3d获取点云分类结果------------------------
segment = []  # 存储分割结果的容器
for i in range(n_clusters_):
    ind = np.where(labels == i)[0]
    clusters_cloud = pcd.select_by_index(ind)
    r_color = np.random.uniform(0, 1, (1, 3))  # 直线点云随机赋色
    clusters_cloud.paint_uniform_color([r_color[:, 0], r_color[:, 1], r_color[:, 2]])
    segment.append(clusters_cloud)
    file_name = "mean_shift_cluster" + str(i + 1) + ".pcd"
    o3d.io.write_point_cloud(file_name, clusters_cloud)
# ----------------------------结果可视化------------------------------
o3d.visualization.draw_geometries(segment, window_name="MeanShift点云聚类",
                                  width=1024, height=768,
                                  left=50, top=50,
                                  mesh_show_back_face=False)

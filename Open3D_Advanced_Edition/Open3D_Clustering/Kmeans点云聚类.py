"""
Author: Yixuan Su
Date: 2025/02/16 17:50
File: Kmeans点云聚类.py
Description: 

"""
import numpy as np
import open3d as o3d
from sklearn.cluster import KMeans
from sklearn.preprocessing import StandardScaler

# ----------------------------加载点云数据---------------------------------
pcd = o3d.io.read_point_cloud("data/two_tree.pcd")
X = np.asarray(pcd.points)
# ----------------------------进行数据训练---------------------------------
# 第一种方式
# 标准化
transfer = StandardScaler()
data_new = transfer.fit_transform(X)
# ----------------------------Kmeans聚类----------------------------------
km = KMeans(n_clusters=2)
km.fit(data_new)
labels = km.predict(data_new)
# 第二种方式
# 不预测
# cluster = KMeans(n_clusters = 3).fit(data_new)
# labels = cluster.labels_s
# 质心
cluster_centers = km.cluster_centers_
labels_unique = np.unique(labels)
n_clusters_ = len(labels_unique)
print("聚类个数为 : %d" % n_clusters_)

# ------------------------使用open3d获取点云分类结果----------------------
segment = []  # 存储分割结果的容器
for i in range(n_clusters_):
    ind = np.where(labels == i)[0]
    clusters_cloud = pcd.select_by_index(ind)
    r_color = np.random.uniform(0, 1, (1, 3))
    clusters_cloud.paint_uniform_color([r_color[:, 0], r_color[:, 1], r_color[:, 2]])
    segment.append(clusters_cloud)
    file_name = "kmeans_cluster" + str(i + 1) + ".pcd"
    o3d.io.write_point_cloud(file_name, clusters_cloud)
# -----------------------------结果可视化-------------------------------
o3d.visualization.draw_geometries(segment, window_name="Kmeans点云聚类",
                                  width=1024, height=768,
                                  left=50, top=50,
                                  mesh_show_back_face=False)

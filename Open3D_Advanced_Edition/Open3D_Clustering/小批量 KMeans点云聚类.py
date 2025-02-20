"""
Author: Yixuan Su
Date: 2025/02/16 17:52
File: 小批量 KMeans点云聚类.py
Description: 

"""
import time
import numpy as np
import open3d as o3d
from sklearn.cluster import MiniBatchKMeans

# ----------------------------加载点云数据---------------------------------
pcd = o3d.io.read_point_cloud("data/two_tree.pcd")
X = np.asarray(pcd.points)

# -------------------------MiniBatchKMeans聚类-----------------------------
t0 = time.time()
mbk = MiniBatchKMeans(
    init="k-means++",
    n_clusters=2,
    batch_size=2049,
    n_init=10,
    max_no_improvement=10,
    verbose=0,
)

mbk.fit(X)
labels = mbk.predict(X)
t_mini_batch = time.time() - t0
labels_unique = np.unique(labels)
n_clusters_ = len(labels_unique)
print("聚类个数为 : %d" % n_clusters_)
print("聚类用时： %.3f sec.\n" % t_mini_batch)
# ------------------------使用open3d获取点云分类结果----------------------
segment = []  # 存储分割结果的容器
for i in range(n_clusters_):
    ind = np.where(labels == i)[0]
    clusters_cloud = pcd.select_by_index(ind)
    r_color = np.random.uniform(0, 1, (1, 3))
    clusters_cloud.paint_uniform_color([r_color[:, 0], r_color[:, 1], r_color[:, 2]])
    segment.append(clusters_cloud)
    file_name = "MiniBatchKMeans_cluster" + str(i + 1) + ".pcd"
    o3d.io.write_point_cloud(file_name, clusters_cloud)
# -----------------------------结果可视化-------------------------------
o3d.visualization.draw_geometries(segment, window_name="MiniBatchKMeans点云聚类",
                                  width=1024, height=768,
                                  left=50, top=50,
                                  mesh_show_back_face=False)


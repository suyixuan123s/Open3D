"""
Author: Yixuan Su
Date: 2025/02/19 22:18
File: python  点云Kmean聚类并保存聚类结果.py
Description: 

"""
import numpy as np
import open3d as o3d
from sklearn.cluster import KMeans
from sklearn.preprocessing import StandardScaler
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

# ----------------------------加载点云数据---------------------------------
pcd = o3d.io.read_point_cloud("../data/many_tree.pcd")
data1 = np.asarray(pcd.points)
# ----------------------------进行数据训练---------------------------------
# 第一种方式
# 标准化
transfer = StandardScaler()
data_new = transfer.fit_transform(data1)
# 预估计流程
estimator = KMeans(n_clusters=3)
estimator.fit(data_new)
y_pred = estimator.predict(data_new)
# 第二种方式
# 不预测
# cluster = KMeans(n_clusters = 3).fit(data_new)
# y_pred = cluster.labels_s
# 质心
# centroid = cluster.cluster_centers_
# centroid.shape
# -------------------------使用open3d获取点云分类结果------------------------
idx = np.where(y_pred == 0)[0]  # 获取第一类的索引(第2，3，，，n类获取方式与此相同)
cloud1 = pcd.select_by_index(idx)  # 根据索引提取位于第一类中的点云
# o3d.io.write_point_cloud("111.pcd", cloud1)  # 保存点云
# o3d.visualization.draw_geometries([cloud1])  # 可视化点云
# -------------------------使用绘图工具可视化分类结果-------------------------
fig = plt.figure()

ax = Axes3D(fig)
for i in range(3):
    ax.scatter3D(data_new[y_pred == i, 0], data_new[y_pred == i, 1],
                 data_new[y_pred == i, 2], marker=".")
ax.view_init(elev=60, azim=30)
ax.set_zlabel('Z')
ax.set_ylabel('Y')
ax.set_xlabel('X')
plt.show()

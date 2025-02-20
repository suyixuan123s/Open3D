"""
Author: Yixuan Su
Date: 2025/02/16 17:53
File: 使用PCA将点云投影到主成分空间.py
Description: 

"""
import numpy as np
import open3d as o3d
from sklearn.decomposition import PCA


# ----------------------------加载点云数据---------------------------------
pcd = o3d.io.read_point_cloud("../../../data/bunny.pcd")
X = np.asarray(pcd.points)
# -----------------------------调用PCA------------------------------------
pca = PCA(n_components=3)  # 设置保留主成分个数
pca.fit(X)
# -------------------将数据变换到前三个主成分的方向上------------------------
X_pca = pca.transform(X)
print("Original shape: {}".format(str(X.shape)))  # Original shape
print("Reduced shape: {}".format(str(X_pca.shape)))  # Reduced shape
# 投影后的三维坐标
project_points = X_pca
project_cloud = o3d.geometry.PointCloud()  # 使用numpy生成点云
project_cloud.points = o3d.utility.Vector3dVector(project_points)
project_cloud.colors = pcd.colors  # 获取投影前对应的颜色赋值给投影后的点
o3d.io.write_point_cloud("../../../data/project_cloud.pcd", project_cloud)
o3d.visualization.draw_geometries([project_cloud], window_name="点云投影到主成分空间",
                                  width=900, height=900,
                                  left=50, top=50,
                                  mesh_show_back_face=False)


"""
Author: Yixuan Su
Date: 2025/02/18 15:05
File: Open3D 点云配准-可视化匹配点对之间的连线.py
Description: 

"""
import open3d as o3d

# --------------------------------------------加载点云------------------------------------------------
source = o3d.io.read_point_cloud("../data/1.pcd")
target = o3d.io.read_point_cloud("../data/2.pcd")
# ---------------------------------------K近邻搜索获取匹配点对-----------------------------------------
lines = []  # 用于存储对应点索引的list容器
# # 修改循环范围，确保不会超出 target 的点数
# num_points = min(len(source.points), len(target.points))
# kdtree = o3d.geometry.KDTreeFlann(source)  # 建立 kd-tree 索引
# for idx in range(num_points):
#     [k, nn_indices, _] = kdtree.search_knn_vector_3d(target.points[idx], 1)
#     lines.append([idx, nn_indices[0]])


num_points = len(source.points)
kdtree = o3d.geometry.KDTreeFlann(source)  # 建立kd-tree索引
for idx in range(num_points):
    [k, nn_indices, _] = kdtree.search_knn_vector_3d(target.points[idx], 1)
    lines.append([idx, nn_indices[0]])



# ---------------------------------------------获取直线-----------------------------------------------
line_set = o3d.geometry.LineSet.create_from_point_cloud_correspondences(source, target, lines)
# 给对应线赋色，方法一
line_set.paint_uniform_color([1, 0, 0])
# 给对应线赋色，方法二
# colors = [[1, 0, 0] for i in range(len(lines))]
# line_set.colors = o3d.utility.Vector3dVector(colors)
# ---------------------------------------------结果可视化---------------------------------------------
o3d.visualization.draw_geometries([source, target, line_set])


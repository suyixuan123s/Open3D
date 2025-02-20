"""
Author: Yixuan Su
Date: 2025/02/16 20:03
File: 无序点云平面检测的鲁棒统计方法.py
Description: 

"""

import open3d as o3d

# --------------------------------------加载点云-------------------------------------
dataset = o3d.data.PCDPointCloud()
pcd = o3d.io.read_point_cloud(dataset.path)

# -----------------------------------计算点云法向量----------------------------------
assert (pcd.has_normals())

# -------------------------------------平面检测-------------------------------------
oboxes = pcd.detect_planar_patches(
    normal_variance_threshold_deg=60,  # 设置法线之间容许的方差
    coplanarity_deg=75,  # 控制点距离平面的允许分布
    outlier_ratio=0.75,  # 点到拟和平面的距离阈值
    min_plane_edge_length=0,  # 剔除错误平面边缘点的阈值
    min_num_points=0,  # 八叉树的深度，以及在尝试拟合平面时必须存在的点数
    search_param=o3d.geometry.KDTreeSearchParamKNN(knn=30))

print("Detected {} patches".format(len(oboxes)))

geometries = []
for obox in oboxes:
    mesh = o3d.geometry.TriangleMesh.create_from_oriented_bounding_box(obox, scale=[1, 1, 0.0001])
    mesh.paint_uniform_color(obox.color)
    geometries.append(mesh)
    geometries.append(obox)
geometries.append(pcd)
# ----------------------------------结果可视化---------------------------------------
o3d.visualization.draw_geometries(geometries,
                                  zoom=0.62,
                                  front=[0.4361, -0.2632, -0.8605],
                                  lookat=[2.4947, 1.7728, 1.5541],
                                  up=[-0.1726, -0.9630, 0.2071])

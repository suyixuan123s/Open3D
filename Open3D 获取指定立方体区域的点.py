"""
Author: Yixuan Su
Date: 2025/02/19 19:57
File: Open3D 获取指定立方体区域的点.py
Description: 

"""
import numpy as np
import open3d as o3d


def crop_filter(cloud, min_x=1, max_x=3, min_y=1, max_y=3, min_z=1, max_z=3):
    """
    获取指定区域的点
    :param cloud: 输入点云
    :param min_x: 指定方框的x方向最小值
    :param max_x: 指定方框的x方向最大值
    :param min_y: 指定方框的y方向最小值
    :param max_y: 指定方框的y方向最大值
    :param min_z: 指定方框的z方向最小值
    :param max_z: 指定方框的z方向最大值
    :return: 方框内的点云，方框外的点云
    """
    points = np.asarray(cloud.points)

    ind = np.where((points[:, 0] >= min_x) & (points[:, 0] <= max_x) &
                   (points[:, 1] >= min_y) & (points[:, 1] <= max_y) &
                   (points[:, 2] >= min_z) & (points[:, 2] <= max_z))[0]

    inlier_cloud = cloud.select_by_index(ind)
    outlier_cloud = cloud.select_by_index(ind, invert=True)
    return inlier_cloud, outlier_cloud


# --------------------------------加载点云-----------------------------------
pcd = o3d.io.read_point_cloud("../data/two_tree.pcd")
# --------------------------------裁剪滤波-----------------------------------
in_box_cloud, out_box_cloud = crop_filter(pcd,
                                          min_x=0, max_x=40,  # 给定裁剪方框的范围
                                          min_y=0, max_y=40,
                                          min_z=0, max_z=40)
in_box_cloud.paint_uniform_color([1.0, 0, 0])  # 方框内的点渲染成红色
print(in_box_cloud)  # 位于方框内的点云点的个数
print(out_box_cloud)  # 位于方框外的点云点的个数
# -------------------------------结果可视化----------------------------------
o3d.visualization.draw_geometries([in_box_cloud, out_box_cloud],
                                  window_name="裁剪滤波",
                                  width=800, height=600)

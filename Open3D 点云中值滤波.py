"""
Author: Yixuan Su
Date: 2025/02/18 11:12
File: Open3D 点云中值滤波.py
Description: 

"""
import numpy as np
import open3d as o3d



def median_filter(pcd, radius):
    kdtree = o3d.geometry.KDTreeFlann(pcd)

    points_copy = np.array(pcd.points)
    points = np.asarray(pcd.points)
    num_points = len(pcd.points)

    for i in range(num_points):
        k, idx, _ = kdtree.search_radius_vector_3d(pcd.points[i], radius)
        if k < 3:
            continue

        neighbors = points[idx, :]
        median = np.median(neighbors, 0)

        points_copy[i] = median

    pcd.points = o3d.utility.Vector3dVector(points_copy)


def main():
    # 读取点云，返回 Tensor-based 点云
    pcd = o3d.t.io.read_point_cloud('../data/CSite1_orig-utm.pcd')
    o3d.visualization.draw([pcd])
    #
    # 转换为传统的点云格式
    pcd_legacy = pcd.to_legacy()

    # 执行中值滤波
    median_filter(pcd_legacy, 1)

    # 保存点云
    o3d.t.io.write_point_cloud("test1.pcd", pcd)

    # 可视化滤波后的点云
    o3d.visualization.draw([pcd])


if __name__ == '__main__':
    main()

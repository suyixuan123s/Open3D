"""
Author: Yixuan Su
Date: 2025/02/18 11:24
File: Open3D 点云均值滤波.py
Description: 

"""
import numpy as np
import open3d as o3d


def mean_filter(cloud, radius):
    kdtree = o3d.geometry.KDTreeFlann(cloud)
    points_copy = np.array(cloud.points)
    points = np.asarray(cloud.points)
    num_points = len(cloud.points)

    for i in range(num_points):
        k, idx, _ = kdtree.search_radius_vector_3d(cloud.points[i], radius)
        if k < 3:
            continue

        neighbors = points[idx, :]
        mean = np.mean(neighbors, 0)

        points_copy[i] = mean

    cloud.points = o3d.utility.Vector3dVector(points_copy)


if __name__ == '__main__':
    pcd = o3d.io.read_point_cloud('test1.pcd')
    o3d.visualization.draw_geometries([pcd], window_name="滤波前的点云",
                                      width=1024, height=768,
                                      left=50, top=50,
                                      mesh_show_back_face=False)
    mean_filter(pcd, 0.3)
    o3d.io.write_point_cloud("test.pcd", pcd)
    o3d.visualization.draw_geometries([pcd], window_name="滤波后的点云",
                                      width=1024, height=768,
                                      left=50, top=50,
                                      mesh_show_back_face=False)


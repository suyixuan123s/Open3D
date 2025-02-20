"""
Author: Yixuan Su
Date: 2025/02/19 23:12
File: Open3D 多幅点云配准.py
Description: 

"""

def load_point_clouds(voxel_size=0.0):
    """
    从文件夹中读取三个点云，并下采样
    :param voxel_size: 体素下采样的边长
    :return: 下采样后的点云
    """
    pcds = []
    for i in range(3):
        pcd = o3d.io.read_point_cloud("../data/two_tree.pcd" % i)
        pcd_down = pcd.voxel_down_sample(voxel_size=voxel_size)
        pcds.append(pcd_down)
    return pcds


voxel_size = 0.02  # 体素边长
pcds_down = load_point_clouds(voxel_size)
o3d.visualization.draw_geometries(pcds_down,
                                  zoom=0.3412,
                                  front=[0.4257, -0.2125, -0.8795],
                                  lookat=[2.6172, 2.0475, 1.532],
                                  up=[-0.0694, -0.9768, 0.2024])

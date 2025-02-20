"""
Author: Yixuan Su
Date: 2025/02/19 20:51
File: python  读取、显示、保存las点云.py
Description: 

"""
import laspy
import numpy as np
import open3d as o3d


def las_viewer(las_cloud):
    xyz = np.vstack((las_cloud.x, las_cloud.y, las_cloud.z)).transpose()
    color = np.vstack((las_cloud.red / 65025, las_cloud.green / 65025, las_cloud.blue / 65025)).transpose()
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz)
    pcd.colors = o3d.utility.Vector3dVector(color)
    o3d.visualization.draw_geometries([pcd])


las = laspy.read("las//tree.las")
las_viewer(las)


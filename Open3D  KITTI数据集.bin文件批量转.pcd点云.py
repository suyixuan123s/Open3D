"""
Author: Yixuan Su
Date: 2025/02/18 20:44
File: Open3D  KITTI数据集.bin文件批量转.pcd点云.py
Description: 

"""
import struct
import open3d as o3d
import numpy as np


def read_bin_cloud(path):
    """
    :param path: .bin文件
    :return:     点集数组
    """
    bin_list = []
    with open(path, 'rb') as f:
        content = f.read()
        points_iter = struct.iter_unpack('ffff', content)
        for idx, point in enumerate(points_iter):
            bin_list.append([point[0], point[1], point[2]])
    return np.asarray(bin_list, dtype=np.float32)


if __name__ == "__main__":
    binFile = "0.bin"
    # --------------------------读取.bin文件----------------------------
    bin_pc = read_bin_cloud(binFile)
    # -----------------------转成Open3D支持的点云-----------------------
    pcd = o3d.open3d.geometry.PointCloud()
    pcd.points = o3d.open3d.utility.Vector3dVector(bin_pc)
    # ----------------------------结果保存-----------------------------
    o3d.io.write_point_cloud((binFile.split(".")[0] + '.pcd'), pcd)
    # ---------------------------结果可视化-----------------------------
    o3d.open3d.visualization.draw_geometries([pcd])

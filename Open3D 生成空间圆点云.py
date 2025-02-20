"""
Author: Yixuan Su
Date: 2025/02/18 15:54
File: Open3D 生成空间圆点云.py
Description: 

"""
import numpy as np
import open3d as o3d


def generate_3d_circle(sample_num=100, r=1.8, center=None, n=None):
    """
    生层空间圆点云
    :param sample_num: 点的个数
    :param r: 圆半径
    :param center: 圆心坐标
    :param n: 法向量
    :return: 点云
    """
    if n is None:
        n = [1, 0, 0]
    if center is None:
        center = [0, 0, 0]
    angle = np.linspace(-np.pi, np.pi, sample_num)

    a = np.cross(n, [1, 0, 0])  # np.cross(), 向量叉积
    if np.all(a == 0):  # a 是否为0向量
        a = np.cross(n, [0, 1, 0])

    b = np.cross(n, a)
    # 归一化a，b（圆面两个互垂直向量）
    a = a / np.linalg.norm(a)
    b = b / np.linalg.norm(b)
    # 利用空间圆的参数方程生成圆
    c1 = center[0] * np.ones((sample_num, 1))
    c2 = center[1] * np.ones((sample_num, 1))
    c3 = center[2] * np.ones((sample_num, 1))

    c_x = c1 + r * (a[0] * np.cos(angle) + b[0] * np.sin(angle))
    c_y = c2 + r * (a[1] * np.cos(angle) + b[1] * np.sin(angle))
    c_z = c3 + r * (a[2] * np.cos(angle) + b[2] * np.sin(angle))
    # 生成点云
    xyz = np.c_[c_x[0], c_y[0], c_z[0]]
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz)

    return pcd


cloud = generate_3d_circle(center=[2, 2, 0], n=[-1, 1, 0])
o3d.io.write_point_cloud("sync.ply", cloud)
o3d.visualization.draw_geometries([cloud], window_name="生成空间圆点云",
                                  width=900, height=900,
                                  left=50, top=50,
                                  mesh_show_back_face=False)

"""
Author: Yixuan Su
Date: 2025/02/19 20:46
File: Open3D 最小二乘拟合空间球.py
Description: 

"""
import open3d as o3d
import numpy as np
import scipy.optimize as opt


def spherrors(para, points):
    """球面拟合误差"""
    a, b, c, r = para
    x = points[0, :]
    y = points[1, :]
    z = points[2, :]
    return pow((x - a), 2) + pow((y - b), 2) + pow((z - c), 2) - pow(r, 2)


def sphere_fit(point):
    """线性最小二乘拟合"""

    tparas = opt.leastsq(spherrors, [1, 1, 1, 1], point.T, full_output=1)
    paras = tparas[0]
    sphere_r = abs(paras[3])
    sphere_o = [paras[0], paras[1], paras[2]]
    # 计算球度误差
    es = np.mean(np.abs(tparas[2]['fvec'])) / paras[3]  # 'fvec'即为spherrors的值
    return sphere_o, sphere_r


if __name__ == '__main__':
    pcd = o3d.io.read_point_cloud("s2.pcd")
    point3d = np.asarray(pcd.points)
    center, R = sphere_fit(point3d)
    print("球心坐标:%s" % center)
    print("球体半径:%s" % R)
    # 可视化点云与球面拟合结果
    sphere_surface = o3d.geometry.TriangleMesh.create_sphere(radius=R, resolution=40)
    sphere_surface.compute_vertex_normals()
    sphere_surface.paint_uniform_color([0.0, 1.0, 0.0])
    sphere_surface = sphere_surface.translate((center[0], center[1], center[2]))
    o3d.visualization.draw_geometries([pcd, sphere_surface])

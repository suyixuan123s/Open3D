"""
Author: Yixuan Su
Date: 2025/02/19 20:45
File: Open3D 非线性最小二乘拟合空间球.py
Description: 

"""
import open3d as o3d
import numpy as np
import scipy.optimize as opt


def spherical_function(point, a, b, c, r):
    """
    构造球面拟合函数，计算以a,b,c,r为参数的球和原始数据之间的误差
    """
    x = point[0, :]
    y = point[1, :]
    z = point[2, :]
    return pow((x - a), 2) + pow((y - b), 2) + pow((z - c), 2) - pow(r, 2)


def sphere_fit(point):
    """"非线性最小二乘拟合"""
    best = np.zeros(point.shape[0])
    popt, pcov = opt.curve_fit(spherical_function, point.T, best, p0=[4.0, 1.4, -4.5, 0.1])
    sphere_r = abs(popt[3])
    sphere_o = [popt[0], popt[1], popt[2]]
    return sphere_o, sphere_r


if __name__ == '__main__':
    pcd = o3d.io.read_point_cloud("../data/2.pcd")
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

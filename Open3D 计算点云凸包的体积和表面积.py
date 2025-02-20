"""
Author: Yixuan Su
Date: 2025/02/18 20:16
File: Open3D 计算点云凸包的体积和表面积.py
Description: 

"""
import open3d as o3d

pcd = o3d.io.read_point_cloud("../data/bunny.pcd")
print(pcd)  # 输出点云点的个数

hull, idx = pcd.compute_convex_hull()
hull.paint_uniform_color([1, 0.7, 0])  # 给凸包渲染颜色
area = hull.get_surface_area()  # 计算表面积
volume = hull.get_volume()  # 计算体积
print("表面积为：", area)
print("体积为：", volume)
o3d.visualization.draw_geometries([pcd, hull])

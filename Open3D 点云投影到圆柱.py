"""
Author: Yixuan Su
Date: 2025/02/18 15:09
File: Open3D 点云投影到圆柱.py
Description: 

"""

import open3d as o3d
import numpy as np

if __name__ == "__main__":
    # ----------------------------------加载点云------------------------------------
    pcd = o3d.io.read_point_cloud("../data/tree2.pcd")
    o3d.visualization.draw_geometries([pcd])

    # -------------------------------获取圆柱模型系数--------------------------------
    cylinder_pt = np.array([0.0, 0.0, 0.0])  # 圆柱轴线上一点
    cylinder_dir = np.array([1.0, 0.0, 0.0])  # 圆柱轴向向量
    r = 5.1674  # 圆柱半径

    # 判断圆柱轴线方向向量是否为单位向量，如果不是则进行归一化
    if np.linalg.norm(cylinder_dir) != 1.0:
        cylinder_dir = cylinder_dir / np.linalg.norm(cylinder_dir)

    # --------------------------------投影后的点云----------------------------------
    project_cloud = o3d.geometry.PointCloud(pcd)
    points = np.asarray(pcd.points)

    # 对每个点进行操作
    for i in range(len(pcd.points)):
        w = points[i]
        k = np.dot((w - cylinder_pt), cylinder_dir)
        p = cylinder_pt + k * cylinder_dir
        pdir = (w - p) / np.linalg.norm(w - p)
        pro = p + pdir * r
        project_cloud.points[i] = pro

    # --------------------------------结果可视化------------------------------------
    o3d.visualization.draw_geometries([project_cloud], window_name="点云投影到圆柱",
                                      width=1024, height=768,
                                      left=50, top=50,
                                      mesh_show_back_face=False)
    o3d.io.write_point_cloud("project_cloud.pcd", project_cloud)

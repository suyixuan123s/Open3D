"""
Author: Yixuan Su
Date: 2025/02/18 16:04
File: Open3D 点云投影到直线.py
Description: 

"""
import open3d as o3d
import numpy as np

if __name__ == "__main__":
    # ----------------------------------加载点云------------------------------------
    pcd = o3d.io.read_point_cloud("../data/1.pcd")
    o3d.visualization.draw_geometries([pcd])
    # -------------------------------获取直线模型系数--------------------------------
    line_pt = np.array([34.3559, 47.1659, 1.6759])         # 直线上一点
    line_dir = np.array([-0.231855, 0.972481, 0.0229066])  # 直线轴向的单位法向量
    # --------------------------------投影后的点云----------------------------------
    line_dir = line_dir.reshape(1, 3)
    project_cloud = o3d.geometry.PointCloud(pcd)
    pt = np.asarray(project_cloud.points)
    k = (pt - line_pt) @ line_dir.T
    points = line_pt + k @ line_dir
    project_cloud.points = o3d.utility.Vector3dVector(points)
    # --------------------------------结果可视化------------------------------------
    o3d.visualization.draw_geometries([project_cloud], window_name="点云投影到直线",
                                      width=1024, height=768,
                                      left=50, top=50,
                                      mesh_show_back_face=False)


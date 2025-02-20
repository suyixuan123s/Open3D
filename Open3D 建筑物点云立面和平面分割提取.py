"""
Author: Yixuan Su
Date: 2025/02/18 21:05
File: Open3D 建筑物点云立面和平面分割提取.py
Description: 

"""
import open3d as o3d
import numpy as np

# ------------------------------------------------加载点云------------------------------------------------
pcd = o3d.io.read_point_cloud("../data/touyingdianceshi.pcd")
o3d.visualization.draw_geometries([pcd])
# -----------------------------------------------计算法向量-----------------------------------------------
if not pcd.has_normals:
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.3, max_nn=100))
normal = np.asarray(pcd.normals)
nz = normal[:, 2]
# ----------------------------------------------提取平面和立面--------------------------------------------
plane_index = np.where(abs(nz) > 0.95)[0]
facade_index = np.where((nz > -0.05) & (nz < 0.05))[0]
print(facade_index)
# ---------------------------------------------根据索引提取点云-------------------------------------------
plane_cloud = pcd.select_by_index(plane_index)
facade_cloud = pcd.select_by_index(facade_index)
# -----------------------------------------------保存提取结果--------------------------------------------

pcd_folder_path = "../data/"
o3d.io.write_point_cloud(pcd_folder_path + "demo.pcd", plane_cloud)
o3d.io.write_point_cloud(pcd_folder_path + "demo2.pcd", facade_cloud)
# -------------------------------------------------结果可视化--------------------------------------------
facade_cloud.paint_uniform_color([1.0, 0.0, 0.5])
plane_cloud.paint_uniform_color([0.0, 1.0, 0.5])
o3d.visualization.draw_geometries([facade_cloud, plane_cloud], point_show_normal=False, window_name="立面和平面提取",
                                  width=1024, height=768,
                                  left=50, top=50,
                                  mesh_show_back_face=False)  # 可视化点云和法线

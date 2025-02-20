"""
Author: Yixuan Su
Date: 2025/02/11 15:09
File: Open3D_Basic_Edition 计算FPFH特征.py
Description:
"""

import open3d as o3d

# -------------传入点云数据，计算FPFH------------
def fpfh_compute(pcd):
    radius_normal = 0.01  # kdtree参数，用于估计法线的半径，
    print(":: Estimate normal with search radius %.3f." % radius_normal)
    pcd.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))
    # 估计法线的1个参数，使用混合型的kdtree，半径内取最多30个邻居
    radius_feature = 0.02  # kdtree参数，用于估计FPFH特征的半径
    print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(pcd,
                                                               o3d.geometry.KDTreeSearchParamHybrid(
                                                                   radius=radius_feature,
                                                                   max_nn=100))  # 计算FPFH特征,搜索方法kdtree
    return pcd_fpfh  # 返回FPFH特征


# ----------------读取点云数据--------------
source = o3d.io.read_point_cloud("../../data/飞机.pcd")
# -----------------计算的FPFH---------------
source_fpfh = fpfh_compute(source)
print(source_fpfh)


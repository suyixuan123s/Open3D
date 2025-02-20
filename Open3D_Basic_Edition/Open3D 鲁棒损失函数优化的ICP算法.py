"""
Author: Yixuan Su
Date: 2025/02/11 16:10
File: Open3D_Basic_Edition 鲁棒损失函数优化的ICP算法.py
Description: 

"""
import copy
import open3d as o3d
import numpy as np
# -------------------读取点云数据--------------------
source = o3d.io.read_point_cloud("../../data/1.pcd")
target = o3d.io.read_point_cloud("../../data/2.pcd")

# source = o3d.io.read_point_cloud(
#     r'E:\ABB-Project\ABB_wrs\suyixuan\ABB\Pose_Estimation\Task6_Point_Cloud_Segment_and_Analyze\Dataset_Point_Cloud\normalized_realsense_point_cloud.ply')
# target = o3d.io.read_point_cloud(
#     r'E:\ABB-Project\ABB_wrs\suyixuan\ABB\Pose_Estimation\Task9_Normalization_point_cloud\normalize_point_cloud_uniform_scaling\stl_to_point_cloud_voxelization1_estimate_normals_normalized_uniform.ply')

# --------------------计算法向量---------------------
source.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.01, max_nn=30))
target.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.01, max_nn=30))
# ----------------可视化点云初始位置-----------------
o3d.visualization.draw_geometries([source, target], width=600, height=600, mesh_show_back_face=False)
threshold = 1  # 距离阈值
trans_init = np.asarray([[0.862, 0.011, -0.507, 0.5],
                         [-0.139, 0.967, -0.215, 0.7],
                         [0.487, 0.255, 0.835, -1.4],
                         [0.0, 0.0, 0.0, 1.0]])  # 初始变换矩阵，一般由粗配准提供
print("Initial alignment")
evaluation = o3d.pipelines.registration.evaluate_registration(source, target, threshold, trans_init)
print(evaluation)  # 这里输出的是初始位置的 fitness和RMSE
print("Apply point-to-plane Open3D_ICP_Edition")
loss = o3d.pipelines.registration.L2Loss()  # ***************鲁棒核函数的调用方法******************
icp_p2plane = o3d.pipelines.registration.registration_icp(
        source, target, threshold, trans_init,
        o3d.pipelines.registration.TransformationEstimationPointToPlane(loss),    # **********调用方法***********
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=30))  # 设置最大迭代次数
print(icp_p2plane)  # 输出ICP相关信息
print("Transformation is:")
print(icp_p2plane.transformation)  # 输出变换矩阵


# -----------------可视化配准结果---------------------
def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)         # 由于函数transformand paint_uniform_color会更改点云，
    target_temp = copy.deepcopy(target)         # 因此调用copy.deepcoy进行复制并保护原始点云。
    source_temp.paint_uniform_color([1, 0, 0])  # 点云着色
    target_temp.paint_uniform_color([0, 1, 0])
    source_temp.transform(transformation)
    o3d.io.write_point_cloud("../../data/trans_of_source1.pcd", source_temp)  # 保存点云
    o3d.visualization.draw_geometries([source_temp, target_temp], width=600, height=600, mesh_show_back_face=False)


draw_registration_result(source, target, icp_p2plane.transformation)



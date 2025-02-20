"""
Author: Yixuan Su
Date: 2025/02/11 14:09
File: Open3D_Basic_Edition RANSAC实现点云粗配准.py
Description: 

"""
import time
import open3d as o3d
import copy


# -------------传入点云数据，计算FPFH------------
def FPFH_Compute(pcd):
    # 计算法线 其中，估计法线的1个参数，使用混合型的kdtree，半径内取最多30个邻居
    radius_normal = 0.01  # kdtree参数，用于估计法线的半径，
    print(":: Estimate normal with search radius %.3f." % radius_normal)
    pcd.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

    # 计算FPFH特征
    radius_feature = 0.02  # kdtree参数，用于估计FPFH特征的半径
    print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd, o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))  # 计算FPFH特征,搜索方法kdtree
    return pcd_fpfh  # 返回FPFH特征


# ----------------RANSAC配准--------------------
def execute_global_registration(source, target, source_fpfh, target_fpfh):  # 传入两个点云和点云的特征
    distance_threshold = 1  # 设定距离阈值
    print("we use a liberal distance threshold %.3f." % distance_threshold)
    # 2个点云，两个点云的特征，距离阈值，一个函数，4，
    # 一个list[0.9的两个对应点的线段长度阈值，两个点的距离阈值]，
    # 一个函数设定最大迭代次数和最大验证次数
    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source, target, source_fpfh, target_fpfh, True, distance_threshold,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(False), 4,
        [o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
         o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(distance_threshold)
         ], o3d.pipelines.registration.RANSACConvergenceCriteria(4000000, 500))
    return result


# ---------------可视化配准结果----------------
def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)  # 由于函数transformand paint_uniform_color会更改点云，
    target_temp = copy.deepcopy(target)  # 因此调用copy.deepcoy进行复制并保护原始点云。
    source_temp.paint_uniform_color([1, 0, 0])  # 点云着色
    target_temp.paint_uniform_color([0, 1, 0])
    source_temp.transform(transformation)
    o3d.io.write_point_cloud("../../../data/trans_of_source.pcd", source_temp)  # 保存点云
    o3d.visualization.draw_geometries([source_temp, target_temp], width=600, height=600)


# ----------------读取点云数据--------------
source = o3d.io.read_point_cloud("../../../data/1.pcd")
target = o3d.io.read_point_cloud("../../../data/2.pcd")

# source = o3d.io.read_point_cloud(
#     r'E:\ABB-Project\ABB_wrs\suyixuan\ABB\Pose_Estimation\Task6_Point_Cloud_Segment_and_Analyze\Dataset_Point_Cloud\normalized_realsense_point_cloud.ply')
# target = o3d.io.read_point_cloud(
#     r'E:\ABB-Project\ABB_wrs\suyixuan\ABB\Pose_Estimation\Task9_Normalization_point_cloud\normalize_point_cloud_uniform_scaling\stl_to_point_cloud_voxelization1_estimate_normals_normalized_uniform.ply')

print("可视化带匹配的点云")
o3d.visualization.draw_geometries([source, target])

# -----------------计算的FPFH---------------
source_fpfh = FPFH_Compute(source)
target_fpfh = FPFH_Compute(target)

# ---------------调用RANSAC执行配准------------
start = time.time()
result_ransac = execute_global_registration(source, target, source_fpfh, target_fpfh)
print("Global registration took %.3f sec.\n" % (time.time() - start))
print(result_ransac)  # 输出RANSAC配准信息
Tr = result_ransac.transformation
draw_registration_result(source, target, Tr)  # 可视化配准结果

# ------------------ICP配准-------------------
start = time.time()
icp_p2plane = o3d.pipelines.registration.registration_icp(
    source, target, 0.5, result_ransac.transformation,
    o3d.pipelines.registration.TransformationEstimationPointToPlane(),  # 执行点对面的ICP算法
    o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=30))  # 设置最大迭代次数

print("Open3D_ICP_Edition registration took %.3f sec.\n" % (time.time() - start))
print(icp_p2plane)  # 输出ICP相关信息
print("Transformation is:")
print(icp_p2plane.transformation)  # 输出变换矩阵
draw_registration_result(source, target, icp_p2plane.transformation)

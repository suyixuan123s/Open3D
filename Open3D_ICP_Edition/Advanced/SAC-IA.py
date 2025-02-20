import copy
import open3d as o3d
import numpy as np

def preprocess_point_cloud(pcd, voxel_size):
    """
    点云预处理：下采样并计算法线
    Args:
        pcd: 输入点云
        voxel_size: 下采样体素大小
    Returns:
        pcd_down: 下采样后的点云
        pcd_fpfh: 计算的FPFH特征
    """
    # 下采样
    pcd_down = pcd.voxel_down_sample(voxel_size)

    # 估计法线
    pcd_down.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2, max_nn=30)
    )

    # 计算FPFH特征
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down,
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 5, max_nn=100)
    )

    return pcd_down, pcd_fpfh

def execute_global_registration(source_down, target_down, source_fpfh, target_fpfh, voxel_size):
    """
    执行全局配准（基于 RANSAC 和 FPFH 特征）
    Args:
        source_down: 下采样的源点云
        target_down: 下采样的目标点云
        source_fpfh: 源点云的FPFH特征
        target_fpfh: 目标点云的FPFH特征
        voxel_size: 体素大小
    Returns:
        result: 配准结果
    """
    distance_threshold = voxel_size * 1.5  # 设置距离阈值

    # 配置一致性检查器
    checkers = [
        o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),  # 检查边长一致性
        o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(distance_threshold)  # 检查点距离一致性
    ]

    # RANSAC 配准
    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh,
        mutual_filter=True,  # 启用互斥匹配
        max_correspondence_distance=distance_threshold,
        estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        ransac_n=4,  # RANSAC 使用的采样点数
        checkers=checkers,
        criteria=o3d.pipelines.registration.RANSACConvergenceCriteria(max_iteration=400000, confidence=0.999)
    )
    return result

def visualize_registration(source, target, transformation=None, title="Point Cloud Visualization"):
    """
    可视化点云配准结果
    """
    source_copy = copy.deepcopy(source)
    target_copy = copy.deepcopy(target)

    # 使用颜色区分点云
    source_copy.paint_uniform_color([1, 0, 0])  # 红色：源点云
    target_copy.paint_uniform_color([0, 1, 0])  # 绿色：目标点云

    if transformation is not None:
        source_copy.transform(transformation)  # 应用变换矩阵

    print(f"{title}")
    o3d.visualization.draw_geometries([source_copy, target_copy])

def main():
    # 加载点云
    source = o3d.io.read_point_cloud(
        r"/suyixuan/ABB/Pose_Estimation/Task6_Point_Cloud_Segment_and_Analyze/Dataset_Point_Cloud/normalized_realsense_point_cloud.ply")  # 读取源点云
    target = o3d.io.read_point_cloud(
        r"/suyixuan/ABB/Pose_Estimation/Task8_STL_To_Point_Cloud/Datasets/stl_to_point_cloud_voxelization_normalized.ply")  # 读取目标点云

    # 可视化配准前的点云
    print("Visualizing point clouds before registration...")
    visualize_registration(source, target, title="Before Registration")

    # 点云预处理（下采样+FPFH计算）
    voxel_size = 1.0  # 根据点云密度调整
    source_down, source_fpfh = preprocess_point_cloud(source, voxel_size)
    target_down, target_fpfh = preprocess_point_cloud(target, voxel_size)

    # 执行SAC-IA配准
    result = execute_global_registration(source_down, target_down, source_fpfh, target_fpfh, voxel_size)

    # 打印配准信息
    print("\nRegistration Information:")
    print(result)
    print(f"\nFitness: {result.fitness:.4f}")  # 配准得分
    print(f"Inlier RMSE: {result.inlier_rmse:.4f}")  # 内点均方根误差

    # 输出变换矩阵
    print("\nTransformation Matrix:")
    print(result.transformation)

    # 可视化配准后的点云
    print("Visualizing point clouds after registration...")
    visualize_registration(source, target, result.transformation, title="After Registration")

if __name__ == "__main__":
    main()

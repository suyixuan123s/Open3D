import open3d as o3d
import numpy as np
import copy


def divide_scene_into_local_maps(source_pcd, num_divisions):
    """
    将源点云划分为多个局部地图
    Args:
        source_pcd: 源点云
        num_divisions: 划分数量
    Returns:
        local_maps: 划分后的局部地图列表
    """
    points = np.asarray(source_pcd.points)
    step = len(points) // num_divisions
    local_maps = [
        source_pcd.select_by_index(range(i * step, (i + 1) * step))
        for i in range(num_divisions)
    ]
    return local_maps


def preprocess_point_cloud(pcd, voxel_size):
    """
    点云预处理：下采样并计算法线
    Args:
        pcd: 输入点云
        voxel_size: 下采样体素大小
    Returns:
        pcd_down: 下采样后的点云
    """
    # 下采样
    pcd_down = pcd.voxel_down_sample(voxel_size)

    # 估计法线
    pcd_down.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2, max_nn=30)
    )

    return pcd_down


def register_object_to_local_maps(target_pcd, local_maps, voxel_size):
    """
    对目标物体与每个局部地图进行配准，找到最佳匹配的局部地图
    Args:
        target_pcd: 目标物体点云
        local_maps: 局部地图列表
        voxel_size: 配准时使用的体素大小
    Returns:
        best_transformation: 最佳配准的变换矩阵
        best_local_map: 最佳匹配的局部地图
    """
    best_fitness = 0
    best_transformation = None
    best_local_map = None

    initial_guess = np.array([[-0.82647682, -0.06199939, 0.55954637, 0.78003854],
                              [-0.56274889, 0.06308269, -0.82421736, 1.22544392],
                              [0.01580328, -0.99608065, -0.08702647, 0.77418869],
                              [0, 0, 0, 1]])

    # 对每个局部地图计算法线
    for local_map in local_maps:
        local_map.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2, max_nn=30)
        )

        # 使用 Open3D_ICP_Edition 进行配准
        result = o3d.pipelines.registration.registration_icp(
            target_pcd, local_map, max_correspondence_distance=voxel_size * 1.5, init=initial_guess,
            estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            criteria=o3d.pipelines.registration.ICPConvergenceCriteria(
                max_iteration=30000,  # 最大迭代次数
                relative_fitness=1e-7,  # 相对收敛条件
                relative_rmse=1e-6  # 相对均方根误差收敛条件
            )
        )
        # 记录最佳配准结果
        if result.fitness > best_fitness:
            best_fitness = result.fitness
            best_transformation = result.transformation
            best_local_map = local_map

    return best_transformation, best_local_map


def refine_registration(target_pcd, source_pcd, initial_transform, voxel_size):
    """
    使用 Open3D_ICP_Edition 对最佳初始变换进行进一步优化
    Args:
        target_pcd: 目标物体点云
        source_pcd: 源点云
        initial_transform: 初始变换矩阵
        voxel_size: 配准时使用的体素大小
    Returns:
        refined_result: 优化后的配准结果
    """
    distance_threshold = voxel_size * 0.5  # 优化阈值
    result = o3d.pipelines.registration.registration_icp(
        target_pcd, source_pcd, distance_threshold,
        init=initial_transform,
        estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPlane()
    )
    return result


def main():
    # 加载源点云和目标物体点云
    source_pcd = o3d.io.read_point_cloud(
        r"E:\ABB-Project\ABB_wrs\suyixuan\ABB\Pose_Estimation\Task6_Point_Cloud_Segment_and_Analyze\Dataset_Point_Cloud\normalized_realsense_point_cloud.ply")  # 读取源点云
    target_pcd = o3d.io.read_point_cloud(
        r"E:\ABB-Project\ABB_wrs\suyixuan\ABB\Pose_Estimation\Task6_Point_Cloud_Segment_and_Analyze\Dataset_Point_Cloud\normalized_stl_point_cloud.ply")  # 读取目标点云

    # 设置参数
    voxel_size = 0.05  # 配准体素大小
    num_divisions = 3  # 局部分块数量

    # 划分源点云为局部地图
    print("Dividing source point cloud into local maps...")
    local_maps = divide_scene_into_local_maps(source_pcd, num_divisions)

    # 对目标物体点云进行预处理
    print("Preprocessing target point cloud...")
    target_pcd_down = preprocess_point_cloud(target_pcd, voxel_size)

    # 对目标物体与局部地图进行配准
    print("Registering target point cloud to local maps...")
    best_transformation, best_local_map = register_object_to_local_maps(target_pcd_down, local_maps, voxel_size)

    # 输出最佳配准结果
    print("Best Transformation Matrix:")
    print(best_transformation)

    # 可视化目标物体与最佳匹配局部地图
    target_pcd.transform(best_transformation)
    target_pcd.paint_uniform_color([1, 0, 0])  # 红色：目标物体
    best_local_map.paint_uniform_color([0, 1, 0])  # 绿色：最佳匹配的局部地图
    o3d.visualization.draw_geometries([best_local_map, target_pcd], window_name="Best Local Map Matching")

    # 为源点云计算法线
    print("Estimating normals for the source point cloud...")
    source_pcd.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2, max_nn=30)
    )

    # 对目标物体进行全局优化（可选）
    print("Refining registration...")
    refined_result = refine_registration(target_pcd, source_pcd, best_transformation, voxel_size)

    # 输出优化后的位姿矩阵
    print("Refined Transformation Matrix:")
    print(refined_result.transformation)

    # 可视化优化后的配准结果
    target_pcd.transform(refined_result.transformation)
    source_pcd.paint_uniform_color([0.6, 0.6, 0.6])  # 灰色：源点云
    target_pcd.paint_uniform_color([1, 0, 0])  # 红色：目标物体
    o3d.visualization.draw_geometries([source_pcd, target_pcd], window_name="Refined Registration")

    # 保存结果
    output_path = "../data/aligned_object_to_scene.ply"
    o3d.io.write_point_cloud(output_path, target_pcd)
    print(f"Aligned target point cloud saved to {output_path}")


if __name__ == "__main__":
    main()

import copy
import open3d as o3d
import numpy as np

def preprocess_point_cloud(pcd, voxel_size=1.0):
    """
    点云预处理：下采样和计算法线
    """
    pcd_down = pcd.voxel_down_sample(voxel_size)  # 下采样点云
    pcd_down.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2, max_nn=30))  # 计算法线
    return pcd_down

def ndt_registration(source, target, voxel_size):
    """
    使用 NDT 配准点云
    """
    source_down = preprocess_point_cloud(source, voxel_size)  # 预处理源点云
    target_down = preprocess_point_cloud(target, voxel_size)  # 预处理目标点云

    # 初始化变换矩阵（单位矩阵）
    initial_transformation = np.identity(4)

    # 执行 NDT 配准
    result = o3d.pipelines.registration.registration_ndt(
        source_down, target_down, max_correspondence_distance=voxel_size * 1.5,
        init=initial_transformation,
        estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint()
    )

    # 打印配准过程的信息
    print("\nRegistration Information:")
    print(result)
    print(f"\nFitness: {result.fitness:.4f}")  # 配准得分（匹配点占比）
    print(f"Inlier RMSE: {result.inlier_rmse:.4f}")  # 内点的均方根误差

    return result.transformation

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
    source_cloud = o3d.io.read_point_cloud(
        r"/suyixuan/ABB/Pose_Estimation/Task6_Point_Cloud_Segment_and_Analyze/Dataset_Point_Cloud/normalized_realsense_point_cloud.ply")  # 读取源点云
    target_cloud = o3d.io.read_point_cloud(
        r"/suyixuan/ABB/Pose_Estimation/Task8_STL_To_Point_Cloud/Datasets/stl_to_point_cloud_voxelization_normalized.ply")  # 读取目标点云

    # 可视化配准前的点云
    print("Visualizing point clouds before registration...")
    visualize_registration(source_cloud, target_cloud, title="Before Registration")

    # 配准点云
    voxel_size = 1.0  # 根据点云密度调整
    transformation = ndt_registration(source_cloud, target_cloud, voxel_size)

    # 输出变换矩阵
    print("\nTransformation Matrix:")
    print(transformation)

    # 可视化配准后的点云
    print("Visualizing point clouds after registration...")
    visualize_registration(source_cloud, target_cloud, transformation, title="After Registration")

if __name__ == "__main__":
    main()


#
# """
# E:\ABB-Project\ABB_wrs\venv\Scripts\python.exe E:\ABB-Project\ABB_wrs\suyixuan\ABB\Open3D_ICP_Edition\NDT_Registration.py
# Visualizing point clouds before registration...
# Before Registration
# Traceback (most recent call last):
#   File "E:\ABB-Project\ABB_wrs\suyixuan\ABB\Open3D_ICP_Edition\NDT_Registration.py", line 79, in <module>
#     main()
#   File "E:\ABB-Project\ABB_wrs\suyixuan\ABB\Open3D_ICP_Edition\NDT_Registration.py", line 68, in main
#     transformation = ndt_registration(source_cloud, target_cloud, voxel_size)
#   File "E:\ABB-Project\ABB_wrs\suyixuan\ABB\Open3D_ICP_Edition\NDT_Registration.py", line 24, in ndt_registration
#     result = o3d.pipelines.registration.registration_ndt(
# AttributeError: module 'open3d.cpu.pybind.pipelines.registration' has no attribute 'registration_ndt'
#
# Process finished with exit code 1
#
#
#
#
# """
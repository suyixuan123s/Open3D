import open3d as o3d
import numpy as np

# 6对匹配点的坐标（源点云和目标点云中的对应点）
source_points = np.array([
    [-182.003, 62.792702, 1686.573],
    [-162.384, 59.008663, 1677.843],
    [-171.902, 86.946518, 1682.988],
    [-173.278, 33.512215, 1682.873],
    [-148.135, 58.571487, 1665.412],
    [-64.6281, 59.148327, 1786.538],
])  # 替换为源点云中的匹配点

target_points = np.array([
    [-0.716561, 82.801865, -17.3643],
    [0.072789, 82.626495, 22.204897],
    [-30.1777, 84.104233, -3.335946],
    [29.261644, 83.744194, 0.102028],
    [-3.988894, 83.726555, 28.714842],
    [-2.846399, -80.9027, 31.244841]
])  # 替换为目标点云中的匹配点

# 计算旋转矩阵和平移向量
def compute_transform(source_points, target_points):
    # 计算质心
    centroid_source = np.mean(source_points, axis=0)
    centroid_target = np.mean(target_points, axis=0)

    # 去中心化
    source_centered = source_points - centroid_source
    target_centered = target_points - centroid_target

    # 计算H矩阵
    H = np.dot(source_centered.T, target_centered)

    # 奇异值分解
    U, S, Vt = np.linalg.svd(H)
    R = np.dot(Vt.T, U.T)  # 旋转矩阵

    # 计算平移
    t = centroid_target - np.dot(R, centroid_source)

    # 计算变换矩阵
    transform_matrix = np.eye(4)
    transform_matrix[:3, :3] = R
    transform_matrix[:3, 3] = t

    return transform_matrix

# 获取初始变换矩阵
initial_transform = compute_transform(source_points, target_points)

# 打印初始变换矩阵
print("Initial Transformation Matrix from 6-point matching:")
print(initial_transform)

# 加载点云数据
source = o3d.io.read_point_cloud("data/PointCloud.ply")  # 读取源点云
target = o3d.io.read_point_cloud("data/Meshclone.ply")  # 读取目标点云

# 使用计算出的初始变换矩阵进行ICP配准
max_correspondence_distance = 1  # 最大匹配距离

# 进行ICP配准
reg_icp = o3d.pipelines.registration.registration_icp(
    source, target, max_correspondence_distance, init=initial_transform,  # 使用计算的初始变换矩阵
    estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(),
    criteria=o3d.pipelines.registration.ICPConvergenceCriteria(
        max_iteration=30000,  # 最大迭代次数
        relative_fitness=1e-7,  # 相对收敛条件
        relative_rmse=1e-6  # 相对均方根误差收敛条件
    )
)

# 打印配准结果
print("Fitness:", reg_icp.fitness)  # 配准度（匹配的点对数量比例，越接近 1 越好）
print("RMSE:", reg_icp.inlier_rmse)  # 配准误差（越小越好）

# 打印最终变换矩阵
print(f"Final Transformation Matrix after Open3D_ICP_Edition:")
print(reg_icp.transformation)  # 最终变换矩阵

# 应用变换到源点云，得到对齐后的点云
source_pcd = source.transform(reg_icp.transformation)

# 可视化对齐后的点云
o3d.visualization.draw_geometries([source_pcd, target], window_name="After Open3D_ICP_Edition Registration", width=1024, height=768)

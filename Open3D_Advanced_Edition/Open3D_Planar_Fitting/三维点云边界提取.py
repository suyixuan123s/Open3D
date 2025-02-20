"""
Author: Yixuan Su
Date: 2025/02/16 17:56
File: 三维点云边界提取.py
Description: 

"""
import open3d as o3d
import numpy as np

# -----------------------------------------读取点云---------------------------------------------
pcd = o3d.io.read_point_cloud("../../../data/bunny.pcd")  # 读取点云数据

# 检查点云是否为空
if len(pcd.points) == 0:
    print("Error: The point cloud is empty. Please check the file path and content.")
else:
    # 创建 KDTree 搜索参数
    search_param = o3d.geometry.KDTreeSearchParamKNN(knn=30)  # 使用KNN方式
    pcd.estimate_normals(search_param=search_param, fast_normal_computation=True)

# ----------------------------------------边界点提取（基于法向量的变化）-------------------------
# 获取法向量
normals = np.asarray(pcd.normals)

# 计算每个点的曲率，简单的方法是计算法向量的变化量（差异较大的点可能是边界点）
curvature = np.linalg.norm(normals, axis=1)  # 使用法向量的大小作为曲率的简单估计

# 设置阈值，选择曲率较大的点
threshold = 0.05  # 根据实际点云调整阈值
boundary_indices = np.where(curvature > threshold)[0]

# 提取边界点
boundary_points = pcd.select_by_index(boundary_indices)
boundary_points.paint_uniform_color([1.0, 0.0, 0.0])  # 将边界点着色为红色

# 打印边界点数量
print(f"Detected {len(boundary_points.points)} boundary points from {len(pcd.points)} points.")

# 保存边界点云
o3d.io.write_point_cloud("../../../data/boundary_points.pcd", boundary_points)

# 给原始点云着色
pcd.paint_uniform_color([0.6, 0.6, 0.6])  # 设置原始点云的颜色为灰色

# -----------------------------------------结果可视化--------------------------------------------
vis = o3d.visualization.Visualizer()
vis.create_window(window_name='三维点云边界提取', width=1200, height=800)

# 设置可视化参数
opt = vis.get_render_option()
opt.background_color = np.asarray([1, 1, 1])  # 设置背景色为白色
opt.point_size = 3  # 设置点的大小

# 添加几何体（边界点和原始点云）
vis.add_geometry(boundary_points)
vis.add_geometry(pcd)

# 启动可视化
vis.run()  # 激活显示窗口
vis.destroy_window()  # 销毁窗口

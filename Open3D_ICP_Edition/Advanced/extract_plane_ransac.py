"""
Author: Yixuan Su
Date: 2025/01/03 15:49
File: extract_plane_ransac.py
Description: 
"""
import open3d as o3d

def save_point_cloud(point_cloud, file_path):
    """
    保存点云到指定文件
    Args:
        point_cloud: 点云对象 (open3d.geometry.PointCloud)
        file_path: 保存路径 (字符串)
    """
    o3d.io.write_point_cloud(file_path, point_cloud)
    print(f"Point cloud saved to: {file_path}")

def main():
    # 加载点云
    file_path = r"/suyixuan/Open3D_ICP_Edition/PointCloud_1.ply"  # 替换为你的点云文件路径
    point_cloud = o3d.io.read_point_cloud(file_path)

    # 提取基准平面
    print("Extracting plane using RANSAC...")
    plane_model, inliers = point_cloud.segment_plane(
        distance_threshold=0.01, ransac_n=3, num_iterations=5000
    )

    # 分离平面点和非平面点
    inlier_cloud = point_cloud.select_by_index(inliers)
    outlier_cloud = point_cloud.select_by_index(inliers, invert=True)

    # 设置点云颜色（可选）
    inlier_cloud.paint_uniform_color([1.0, 0, 0])  # 平面点设置为红色
    outlier_cloud.paint_uniform_color([0, 1.0, 0])  # 非平面点设置为绿色

    # 保存平面点和非平面点到 .ply 文件
    inlier_file_path = "../data/output_inliers1.ply"  # 替换为保存平面点云的路径
    outlier_file_path = "../data/output_outliers1.ply"  # 替换为保存非平面点云的路径
    save_point_cloud(inlier_cloud, inlier_file_path)
    save_point_cloud(outlier_cloud, outlier_file_path)

    # 可视化结果
    print("Visualizing point clouds...")
    o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])

if __name__ == "__main__":
    main()


"""
Author: Yixuan Su
Date: 2025/01/03
File: analyze_plane_dimensions.py
Description: Analyze and visualize the dimensions of a plane from point cloud data.
"""
import open3d as o3d
import numpy as np

def analyze_plane_dimensions(plane_cloud):
    """
    Analyze the dimensions of a plane from point cloud data.
    Args:
        plane_cloud: Plane point cloud (open3d.geometry.PointCloud)
    Returns:
        tuple: Width, length, and height of the plane (float, float, float)
    """
    points = np.asarray(plane_cloud.points)
    if points.size == 0:
        raise ValueError("Point cloud is empty.")

    min_bound = np.min(points, axis=0)
    max_bound = np.max(points, axis=0)

    width = max_bound[0] - min_bound[0]  # X-direction
    length = max_bound[1] - min_bound[1]  # Y-direction
    height = max_bound[2] - min_bound[2]  # Z-direction (optional)

    return width, length, height

def main():
    # 硬编码路径
    file_path = r"/suyixuan/Open3D_ICP_Edition/output_inliers1.ply"
    print(f"Loaded file path: {file_path}")

    try:
        point_cloud = o3d.io.read_point_cloud(file_path)
    except Exception as e:
        print(f"Error loading point cloud: {e}")
        return

    if len(point_cloud.points) == 0:
        print("The point cloud is empty.")
        return

    print("Analyzing plane dimensions...")
    try:
        width, length, height = analyze_plane_dimensions(point_cloud)
    except ValueError as e:
        print(e)
        return

    print(f"Plane Dimensions:")
    print(f"Width: {width:.2f}")
    print(f"Length: {length:.2f}")
    print(f"Height (optional): {height:.2f}")

    point_cloud.paint_uniform_color([1.0, 0, 0])  # Set plane color to red
    o3d.visualization.draw_geometries([point_cloud])

if __name__ == "__main__":
    main()

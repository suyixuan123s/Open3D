"""
Author: Yixuan Su
Date: 2025/02/19 23:13
File: Open3D RGBD测程法.py
Description: 

"""
import open3d as o3d
import numpy as np

# ---------------------------------从文件中读取相机轨迹--------------------------------
pinhole_camera_intrinsic = o3d.io.read_pinhole_camera_intrinsic("RGBD/camera_primesense.json")
print(pinhole_camera_intrinsic.intrinsic_matrix)
# -------------------------------------加载数据---------------------------------------
source_color = o3d.io.read_image("RGBD/color/00000.jpg")
source_depth = o3d.io.read_image("RGBD/depth/00000.png")
target_color = o3d.io.read_image("RGBD/color/00001.jpg")
target_depth = o3d.io.read_image("RGBD/depth/00001.png")
source_rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
    source_color, source_depth, convert_rgb_to_intensity=False)
target_rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
    target_color, target_depth, convert_rgb_to_intensity=False)
target_pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
    target_rgbd_image, pinhole_camera_intrinsic)
# ----------------------------根据对应的RGBD图像计算里程--------------------------------
option = o3d.pipelines.odometry.OdometryOption()
odo_init = np.identity(4)
print(option)
# 方法一：FromColorTerm()
[success_color_term, trans_color_term, info] = o3d.pipelines.odometry.compute_rgbd_odometry(
    source_rgbd_image, target_rgbd_image, pinhole_camera_intrinsic, odo_init,
    o3d.pipelines.odometry.RGBDOdometryJacobianFromColorTerm(), option)
# 方法二：FromHybridTerm()
[success_hybrid_term, trans_hybrid_term, info_1] = o3d.pipelines.odometry.compute_rgbd_odometry(
    source_rgbd_image, target_rgbd_image, pinhole_camera_intrinsic, odo_init,
    o3d.pipelines.odometry.RGBDOdometryJacobianFromHybridTerm(), option)
# -----------------------------可视化RGBD图像对--------------------------------------
if success_color_term:
    print("Using RGB-D Odometry")
    print(trans_color_term)
    source_pcd_color_term = o3d.geometry.PointCloud.create_from_rgbd_image(
        source_rgbd_image, pinhole_camera_intrinsic)
    source_pcd_color_term.transform(trans_color_term)
    o3d.visualization.draw_geometries([target_pcd, source_pcd_color_term],
                                      zoom=0.48,
                                      front=[0.0999, -0.1787, -0.9788],
                                      lookat=[0.0345, -0.0937, 1.8033],
                                      up=[-0.0067, -0.9838, 0.1790])
if success_hybrid_term:
    print("Using Hybrid RGB-D Odometry")
    print(trans_hybrid_term)
    source_pcd_hybrid_term = o3d.geometry.PointCloud.create_from_rgbd_image(
        source_rgbd_image, pinhole_camera_intrinsic)
    source_pcd_hybrid_term.transform(trans_hybrid_term)
    o3d.visualization.draw_geometries([target_pcd, source_pcd_hybrid_term],
                                      zoom=0.48,
                                      front=[0.0999, -0.1787, -0.9788],
                                      lookat=[0.0345, -0.0937, 1.8033],
                                      up=[-0.0067, -0.9838, 0.1790])

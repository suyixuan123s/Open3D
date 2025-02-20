"""
Author: Yixuan Su
Date: 2025/02/10 16:12
File: Open3D 深度图像转点云.py
Description: 

"""
import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt

# ------------------------------读取深度图像--------------------------------
depth = o3d.t.io.read_image('../../data/TUM_depth.png')
# ------------------------------设置相机内参--------------------------------
intrinsic = o3d.core.Tensor([[535.4, 0, 320.1], [0, 539.2, 247.6],
                             [0, 0, 1]])
# ------------------------------可视化深度图--------------------------------
fig, axs = plt.subplots()
axs.imshow(np.asarray(depth.to_legacy()))
plt.show()
# ------------------------------深度图转点云--------------------------------
pcd = o3d.t.geometry.PointCloud.create_from_depth_image(depth,
                                                        intrinsic,
                                                        depth_scale=5000.0,
                                                        depth_max=10.0)
# --------------------------------保存结果----------------------------------
o3d.io.write_point_cloud("depth2cloud.pcd", pcd.to_legacy())
# -------------------------------结果可视化---------------------------------
o3d.visualization.draw([pcd])


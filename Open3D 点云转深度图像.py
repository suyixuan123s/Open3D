"""
Author: Yixuan Su
Date: 2025/02/18 10:42
File: Open3D 点云转深度图像.py
Description: 

"""

import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt

# --------------------------------读取点云-----------------------------------
pcd = o3d.t.io.read_point_cloud("../data/depth2cloud.pcd")
# -------------------------------可视化点云----------------------------------
o3d.visualization.draw([pcd])

# -------------------------------设置相机内参--------------------------------
intrinsic = o3d.core.Tensor([[535.4, 0, 320.1], [0, 539.2, 247.6],
                             [0, 0, 1]])
# -------------------------------点云转深度图--------------------------------
depth_reproj = pcd.project_to_depth_image(640,
                                          480,
                                          intrinsic,
                                          depth_scale=5000.0,
                                          depth_max=10.0)
o3d.t.io.write_image("666666.png", depth_reproj)  # 保存图片
# -------------------------------可视化深度图--------------------------------
fig, axs = plt.subplots()
axs.imshow(np.asarray(depth_reproj))
plt.show()



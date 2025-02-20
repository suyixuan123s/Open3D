"""
Author: Yixuan Su
Date: 2025/02/18 21:53
File: Open3D点云转二值图.py
Description: 将点云投影为2D二值图，并进行边缘检测
"""

import open3d as o3d
import numpy as np
import cv2

# --------------------读取点云-------------------------
cloud = o3d.io.read_point_cloud("../data/tree2.pcd")
step = 0.01  # 像素大小
# ------------------创建像素格网-----------------------
point_cloud = np.asarray(cloud.points)
# 1、获取点云数据边界
x_min, y_min, z_min = np.amin(point_cloud, axis=0)
x_max, y_max, z_max = np.amax(point_cloud, axis=0)
# 2、计算像素格网行列数
width = np.ceil((x_max - x_min) / step)
height = np.ceil((y_max - y_min) / step)
print("像素格网的大小为： {} x {}".format(width, height))
# 创建一个黑色的空白图像
img = np.zeros((int(width), int(height)), dtype=np.uint8)
# img.fill(0)  # 设置图片背景颜色，默认为：黑色。
# 3、计算每个点的像素格网索引，并将有点的像素格网赋值为白色
for i in range(len(point_cloud)):
    col = np.floor((point_cloud[i][0] - x_min) / step)
    row = np.floor((point_cloud[i][1] - y_min) / step)
    img[int(col), int(row)] = 255
# 生成的图片与实际视角偏差90°，因此做一下旋转
img90 = np.rot90(img)
canny = cv2.Canny(img90, 100, 200, 3)

# cv2.imwrite("img90.png", img90)  # 保存图片
cv2.imshow('pc2picture', img90)  # 显示图片
cv2.imshow('canny', canny)  # 显示图片
cv2.waitKey(0)


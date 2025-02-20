"""
Author: Yixuan Su
Date: 2025/02/18 21:50
File: Open3D 点云沿坐标轴进行等距切片.py
Description: 

"""
import open3d as o3d
import numpy as np


def visualizer_cloud(filtered):
    # ------------------------显示点云切片---------------------------
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name='点云切片', width=800, height=600)
    # -----------------------可视化参数设置--------------------------
    opt = vis.get_render_option()
    opt.background_color = np.asarray([0, 0, 0])  # 设置背景色*****
    opt.point_size = 1  # 设置点的大小*************
    vis.add_geometry(filtered)  # 加载点云到可视化窗口
    vis.run()  # 激活显示窗口，这个函数将阻塞当前线程，直到窗口关闭。
    vis.destroy_window()  # 销毁窗口，这个函数必须从主线程调用。


# -----------------------------读取点云-----------------------------
cloud = o3d.io.read_point_cloud("../data/tree2.pcd")
# -----------------------------参数设置-----------------------------
Delta = 0.04     # 切片厚度
dPlatform = 0.4  # 相邻切片间距
# 获取点坐标
point_cloud = np.asarray(cloud.points)
# 获取点云数据边界
_, _, z_min = np.amin(point_cloud, axis=0)
# -----------------------------进行切片-----------------------------
idx = []
for i in range(len(point_cloud)):
    # 计算切片间隔数
    index = np.floor((point_cloud[i][2] - z_min) / dPlatform)
    # Z最小值 + 间隔数 * 切片间距 = 下一个切片的最小值
    sliceMin = z_min + index * dPlatform
    if sliceMin <= point_cloud[i][2] < sliceMin + Delta:
        idx.append(i)
# --------------------------提取切片点云----------------------------
slicing_cloud = (cloud.select_by_index(idx))
o3d.io.write_point_cloud("data//slicePointCloud.pcd", slicing_cloud)
# ---------------------------可视化切片-----------------------------
visualizer_cloud(slicing_cloud)

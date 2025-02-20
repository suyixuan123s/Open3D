"""
Author: Yixuan Su
Date: 2025/02/19 23:18
File: Open3D 体素随机下采样.py
Description: 

"""
import open3d as o3d
import numpy as np
import random


# 体素随机下采样，第一个参数为输入的点云，第二个参数为体素分辨率
def voxel_random_filter(cloud, leaf_size):

    point_cloud = np.asarray(cloud.points)
    # 1、计算边界点
    x_min, y_min, z_min = np.amin(point_cloud, axis=0)
    x_max, y_max, z_max = np.amax(point_cloud, axis=0)
    # 2、计算体素格网的维度
    Dx = (x_max - x_min) // leaf_size + 1
    Dy = (y_max - y_min) // leaf_size + 1
    Dz = (z_max - z_min) // leaf_size + 1
    print("Dx x Dy x Dz is {} x {} x {}".format(Dx, Dy, Dz))
    # 3、计算每个点的格网索引
    h = list()  # h 为保存索引的列表
    for i in range(len(point_cloud)):
        hx = (point_cloud[i][0] - x_min) // leaf_size
        hy = (point_cloud[i][1] - y_min) // leaf_size
        hz = (point_cloud[i][2] - z_min) // leaf_size
        h.append(hx + hy * Dx + hz * Dx * Dy)
    h = np.array(h)
    # 4、体素格网内随机筛选点
    h_indice = np.argsort(h)  # 返回h里面的元素按从小到大排序的索引
    h_sorted = h[h_indice]
    random_idx = []
    begin = 0
    for i in range(len(h_sorted) - 1):
        if h_sorted[i] == h_sorted[i + 1]:
            continue
        else:
            point_idx = h_indice[begin: i + 1]
            random_idx.append(random.choice(point_idx))
        begin = i + 1
    filtered_points = (cloud.select_by_index(random_idx))
    return filtered_points


def visualizer_cloud(filtered):
    # ----------------------显示滤波后的点云--------------------------
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name='体素随机下采样结果可视化', width=800, height=600)
    # -----------------------可视化参数设置--------------------------
    opt = vis.get_render_option()
    opt.background_color = np.asarray([0, 0, 0])  # 设置背景色*****
    opt.point_size = 1  # 设置点的大小*************
    opt.show_coordinate_frame = False  # 设置是否添加坐标系
    vis.add_geometry(filtered)  # 加载点云到可视化窗口
    vis.run()  # 激活显示窗口，这个函数将阻塞当前线程，直到窗口关闭。
    vis.destroy_window()  # 销毁窗口，这个函数必须从主线程调用。


if __name__ == '__main__':
    # --------------------------读取点云-----------------------------
    pcd = o3d.io.read_point_cloud("../data/1.pcd")
    # ----------------------调用函数实现下采样-----------------------
    filtered_cloud = voxel_random_filter(pcd, 0.2)  # 第二个参数为体素格网的边长
    print('滤波后的点数为：\n', filtered_cloud)
    visualizer_cloud(filtered_cloud)


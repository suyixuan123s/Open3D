"""
Author: Yixuan Su
Date: 2025/02/18 20:37
File: Open3D 点云快速欧式聚类.py
Description: 

"""
import open3d as o3d
import numpy as np


def fast_euclidean_cluster(cloud, tolerance=0.2, max_n=50, min_cluster_size=100, max_cluster_size=1000):
    """
    快速欧式聚类
    :param cloud:输入点云
    :param max_n:邻域搜索的最大点数
    :param tolerance: 设置近邻搜索的搜索半径（也即两个不同聚类团点之间的最小欧氏距离）
    :param min_cluster_size:设置一个聚类需要的最少的点数目
    :param max_cluster_size:设置一个聚类需要的最大点数目
    :return:聚类结果的索引容器
    """

    num_points = len(cloud.points)
    labels = [0] * num_points  # initalize all point label as 0
    segLab = 1  # Segment label
    kdtree = o3d.geometry.KDTreeFlann(cloud)  # 建立kd-tree索引
    for idx in range(num_points):
        if labels[idx] == 0:  # if Pi.lable =0
            # Pnn = FindNeighbor(pi,dth)
            k, nn_indices, _ = kdtree.search_hybrid_vector_3d(cloud.points[idx], tolerance, max_n)
            minSegLab = segLab

            for j in nn_indices:
                # if Nonzero(Pnn.lab)
                if (labels[j] > 0) and (labels[j] < minSegLab):
                    minSegLab = labels[j]  # minSegLab = min(N0nzero(Pnn.lab),SegLab)
            for j in nn_indices:  # foreach pj in Pnn do
                tempLab = labels[j]
                if tempLab > minSegLab:  # if pj.lab > minSeglab then
                    for k in range(num_points):  # foreach pk.lab in P do
                        if labels[k] == tempLab:  # if pk.lab = Pj.lab then
                            labels[k] = minSegLab  # pk.lab = minSegLab

                labels[j] = minSegLab  # 将所有邻近点分类

            segLab += 1
    # 根据分类结果对点云附加分类标签
    labMap = [[0] * 2 for _ in range(num_points)]
    for iv in range(num_points):
        labMap[iv][0] = iv
        labMap[iv][1] = labels[iv]
    labMap = sorted(labMap, key=lambda x: x[1])
    # 根据分类标签对点云进行分类
    index = 0
    allClusters = []
    for iv in range(num_points):
        seed_queue = []  # 定义一个种子队列
        if labMap[iv][1] != labMap[index][1]:
            for j in range(index, iv):
                seed_queue.append(labMap[j][0])
            index = iv
            if max_cluster_size > len(seed_queue) > min_cluster_size:
                allClusters.append(seed_queue)

    return allClusters


if __name__ == '__main__':
    # --------------------------加载点云数据------------------------------
    pcd = o3d.io.read_point_cloud("../data/many_tree.pcd")
    # --------------------------快速欧式聚类------------------------------
    fec = fast_euclidean_cluster(pcd, tolerance=0.2, max_n=50, min_cluster_size=1000, max_cluster_size=5000)
    # -------------------------聚类结果分类保存---------------------------
    segment = []  # 存储分割结果的容器
    for i in range(len(fec)):
        ind = fec[i]
        clusters_cloud = pcd.select_by_index(ind)
        r_color = np.random.uniform(0, 1, (1, 3))  # 直线点云随机赋色
        clusters_cloud.paint_uniform_color([r_color[:, 0], r_color[:, 1], r_color[:, 2]])
        segment.append(clusters_cloud)
        # file_name = "fast_euclidean_cluster" + str(i + 1) + ".pcd"
        # o3d.io.write_point_cloud(file_name, clusters_cloud)
    # ----------------------------结果可视化------------------------------
    o3d.visualization.draw_geometries(segment, window_name="快速欧式聚类",
                                      width=1024, height=768,
                                      left=50, top=50,
                                      mesh_show_back_face=False)

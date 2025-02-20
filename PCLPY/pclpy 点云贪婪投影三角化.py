"""
Author: Yixuan Su
Date: 2025/02/19 20:09
File: pclpy 点云贪婪投影三角化.py
Description: 

"""
from pclpy import pcl
import numpy as np
import open3d as o3d


def cloud_with_normals(cloud_in):
    """
    合并两个字段，需要两个字段有相同的点数，主要用于合并XYZ坐标和Normal。
    拼接PointXYZ和Normal组成PointNormal点云，这么做是因为：根据C++的PCL使用经验知，
    NormalEstimation直接计算PointNormal不对，需要用这种方式得到PointNormal类型的点云。
    :param cloud_in: 输入PointXYZ格式的点云
    :return: 输出PointNormal格式的点云
    """
    n = pcl.features.NormalEstimationOMP.PointXYZ_Normal()
    # n.setViewPoint(0, 0, 0)  # 设置视点，默认为（0，0，0）
    n.setInputCloud(cloud_in)
    n.setNumberOfThreads(6)
    n.setKSearch(10)  # 点云法向计算时，需要所搜的近邻点大小
    # n.setRadiusSearch(0.03)  # 半径搜素
    cloud_normals = pcl.PointCloud.Normal()
    n.compute(cloud_normals)  # 开始进行法向计

    d = np.hstack((cloud_in.xyz, cloud_normals.normals))  # 拼接字段
    cloud_with_normal = pcl.PointCloud.PointNormal.from_array(d)
    return cloud_with_normal


if __name__ == '__main__':
    # -----------------------------------读取点云----------------------------------
    cloud = pcl.PointCloud.PointXYZ()
    reader = pcl.io.PCDReader()
    reader.read('bunny.pcd', cloud)
    print("点的个数为: ", cloud.size())
    # --------------------计算点云法向量并连接XYZ和法向量字段------------------------
    cloud_normals = cloud_with_normals(cloud)
    # --------------------------------贪婪三角化算法-------------------------------
    tree2 = pcl.search.KdTree.PointNormal()
    tree2.setInputCloud(cloud_normals)
    gp3 = pcl.surface.GreedyProjectionTriangulation.PointNormal()
    triangles = pcl.PolygonMesh()
    gp3.setSearchRadius(0.035)  # 设置连接点之间的最大距离（即三角形的最大边长）
    gp3.setMu(2.5)  # 设置被样本点搜索其临近点的最远距离，为了适应点云密度的变化
    gp3.setMaximumNearestNeighbors(100)  # 设置样本点可搜索的邻域个数
    gp3.setMaximumSurfaceAngle(np.pi / 4)  # 设置某点法线方向偏离样本点法线方向的最大角度
    gp3.setMinimumAngle(np.pi / 18)  # 设置三角化后得到三角形内角的最小角度 10degrees
    gp3.setMaximumAngle(2 * np.pi / 3)  # 设置三角化后得到三角形内角的最大角度 120 degrees
    gp3.setNormalConsistency(False)  # 设置该参数保证法线朝向一致
    # ----------------------------------顶点信息-----------------------------------
    parts = pcl.vectors.Int()
    states = pcl.vectors.Int()
    # --------------------------------获取结果并保存-------------------------------
    gp3.setInputCloud(cloud_normals)  # 设置输入包含法线的点云
    gp3.setSearchMethod(tree2)  # 设置搜索方式
    gp3.reconstruct(triangles)  # 重建提取三角化
    # pcl.io.saveVTKFile('bunny_gp3.vtk', triangles)
    pcl.io.savePLYFile('bunny_gp3.ply', triangles)
    mesh = o3d.io.read_triangle_mesh("bunny_gp3.ply")
    mesh.paint_uniform_color([1, 0.7, 0])  # 给mesh渲染颜色
    o3d.visualization.draw_geometries([mesh], window_name="贪婪三角投影建模",
                                      width=1024, height=768,
                                      left=50, top=50,
                                      mesh_show_wireframe=False,  # 是否以格网线的形式显示
                                      mesh_show_back_face=False  # 是否显示面片背景
                                      )  # 显示mesh模型

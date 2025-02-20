"""
Author: Yixuan Su
Date: 2025/02/18 22:17
File: pclpy 投影滤波器 点云投影到圆柱.py
Description: 

"""
from pclpy import pcl


def point_cloud_viewer(cloud, cloud_filtered):
    # Open 3D viewer and add point cloud and normals
    viewer = pcl.visualization.PCLVisualizer("cloud_viewer")
    v0 = 1
    viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v0)
    viewer.setBackgroundColor(0.0, 0.0, 0.0, v0)
    single_color = pcl.visualization.PointCloudColorHandlerCustom.PointXYZ(cloud, 0.0, 255.0, 0.0)
    viewer.addPointCloud(cloud, single_color, "sample cloud1", v0)

    v1 = 2
    viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v1)
    viewer.setBackgroundColor(0.0, 0.0, 0.0, v1)
    single_color = pcl.visualization.PointCloudColorHandlerCustom.PointXYZ(cloud_filtered, 255.0, 0.0, 0.0)
    viewer.addPointCloud(cloud_filtered, single_color, "sample cloud2", v1)

    viewer.setPointCloudRenderingProperties(0, 1, "sample cloud1", v0)
    viewer.setPointCloudRenderingProperties(0, 1, "sample cloud2", v1)
    viewer.addCoordinateSystem(1.0)
    while not viewer.wasStopped():
        viewer.spinOnce(10)


# ----------------------------------加载点云------------------------------------
cloud = pcl.PointCloud.PointXYZ()
reader = pcl.io.PCDReader()
reader.read("sn.pcd", cloud)
print('点云中有: ', cloud.size(), '个点')
# ----------------------------------估计法线------------------------------------
ne = pcl.features.NormalEstimation.PointXYZ_Normal()
ne.setInputCloud(cloud)  # 设置输入点云
tree = pcl.search.KdTree.PointXYZ()  # 构建KD树
ne.setSearchMethod(tree)  # 设置近邻搜索方式为KD树
ne.setKSearch(50)  # 近邻搜索点数
normals = pcl.PointCloud.Normal()  # 法向量
ne.compute(normals)  # 计算法向量
# ----------------------------RANSAC拟合分割圆柱--------------------------------
seg = pcl.segmentation.SACSegmentationFromNormals.PointXYZ_Normal()
seg.setOptimizeCoefficients(True)  # 设置对估计的模型系数需要进行优化
seg.setInputCloud(cloud)  # 设置输入点云
seg.setInputNormals(normals)  # 设置输入法向量
seg.setModelType(5)  # 创建圆柱体分割对象
seg.setMethodType(0)  # 设置采用RANSAC算法进行参数估计
seg.setNormalDistanceWeight(0.3)  # 设置表面法线权重系数
seg.setMaxIterations(10000)  # 设置迭代的最大次数
seg.setDistanceThreshold(0.1)  # 设置内点到模型距离的最大值
seg.setRadiusLimits(2.0, 3.0)  # 设置圆柱模型半径的范围
# ----------------------------获取椭圆内点和参数---------------------------------
coefficients_cylinder = pcl.ModelCoefficients()
inliers_cylinder = pcl.PointIndices()
seg.segment(inliers_cylinder, coefficients_cylinder)
print('圆柱参数为: ', coefficients_cylinder.values)
# ------------------------------投影到圆柱--------------------------------------
proj = pcl.filters.ProjectInliers.PointXYZ()
proj.setInputCloud(cloud)
proj.setModelType(5)
proj.setModelCoefficients(coefficients_cylinder)
cloud_filtered = pcl.PointCloud.PointXYZ()
proj.filter(cloud_filtered)

point_cloud_viewer(cloud, cloud_filtered)

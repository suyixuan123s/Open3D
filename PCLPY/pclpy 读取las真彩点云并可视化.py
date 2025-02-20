"""
Author: Yixuan Su
Date: 2025/02/19 19:35
File: pclpy 读取las真彩点云并可视化.py
Description: 

"""
from pclpy import pcl
import laspy
import numpy as np

pc = laspy.read("40m1.las")
xyz = np.vstack((pc.x, pc.y, pc.z)).transpose()
rgb = np.vstack((pc.red / 256, pc.green / 256, pc.blue / 256)).transpose()
cloud = pcl.PointCloud.PointXYZRGB.from_array(xyz, rgb)

viewer = pcl.visualization.PCLVisualizer("3D viewer")
viewer.setBackgroundColor(0, 0, 0)
rgb = pcl.visualization.PointCloudColorHandlerRGBField.PointXYZRGB(cloud)

viewer.addPointCloud(cloud, rgb, "sample cloud")
viewer.setPointCloudRenderingProperties(0, 1, "sample cloud")
viewer.addCoordinateSystem(1)
viewer.initCameraParameters()
while not viewer.wasStopped():
    viewer.spinOnce(10)


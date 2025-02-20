"""
Author: Yixuan Su
Date: 2025/02/18 15:53
File: Open3D 生成空间3D椭圆点云.py
Description: 

"""
import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt


def generate_3d_ellipse(sample_num=100, a=1, b=2):
    """
    生成空间椭圆
    :param sample_num: 点的个数
    :param a: 长半轴
    :param b: 短半轴
    :return: 点云
    """
    t = np.linspace(0, 2 * np.pi, sample_num)
    x = a * np.cos(t)
    y = b * np.sin(t)
    z = t * 0
    # 生成点云
    xyz = np.c_[x, y, z]
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(xyz)
    return cloud


pcd = generate_3d_ellipse()
# ------------------------------------结果可视化-----------------------------------
points = np.asarray(pcd.points)
x = points[:, 0]
y = points[:, 1]
plt.title('Generate 3D Ellipse')
plt.plot(x, y, 'ro', mec='b', mew=1)  # 绘制椭圆
# draw
plt.xlabel('x')
plt.ylabel('y')
plt.show()  # 真正显示出上述的绘图结果
o3d.visualization.draw_geometries([pcd], window_name="生成空间椭圆点云",
                                  width=900, height=900,
                                  left=50, top=50,
                                  mesh_show_back_face=False)

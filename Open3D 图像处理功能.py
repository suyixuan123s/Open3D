"""
Author: Yixuan Su
Date: 2025/02/19 20:00
File: Open3D 图像处理功能.py
Description: 

"""
# 图像翻转
# import open3d as o3d
#
# img = o3d.io.read_image("../data/lucy.jpg")  # 读取图像
# o3d.visualization.draw_geometries([img])  # 可视化图像
# img1 = img.flip_horizontal()  # 左右翻转
# o3d.visualization.draw_geometries([img1])  # 可视化图像
# img2 = img.flip_vertical()    # 上下翻转
# o3d.visualization.draw_geometries([img2])  # 可视化图像


# 图像滤波

import open3d as o3d
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

if __name__ == "__main__":

    print("测试基本图像处理模块")
    # im_raw = mpimg.imread("jt.jpg")
    # im = o3d.geometry.Image(im_raw)
    im = o3d.io.read_image("../data/lucy.jpg")  # 读取图像
    o3d.visualization.draw_geometries([im])  # 可视化图像

    # -------------------------图像Gaussian滤波----------------------------------
    im_g3 = im.filter(o3d.geometry.ImageFilterType.Gaussian3)
    im_g5 = im.filter(o3d.geometry.ImageFilterType.Gaussian5)
    im_g7 = im.filter(o3d.geometry.ImageFilterType.Gaussian7)
    im_gaussian = [im, im_g3, im_g5, im_g7]

    # ---------------------------图像金字塔操作-----------------------------------
    pyramid_levels = 4
    pyramid_with_gaussian_filter = True
    im_pyramid = im.create_pyramid(pyramid_levels, pyramid_with_gaussian_filter)
    im_dx = im.filter(o3d.geometry.ImageFilterType.Sobel3dx)
    im_dx_pyramid = o3d.geometry.Image.filter_pyramid(
        im_pyramid, o3d.geometry.ImageFilterType.Sobel3dx)
    im_dy = im.filter(o3d.geometry.ImageFilterType.Sobel3dy)
    im_dy_pyramid = o3d.geometry.Image.filter_pyramid(
        im_pyramid, o3d.geometry.ImageFilterType.Sobel3dy)
    switcher = {
        0: im_gaussian,
        1: im_pyramid,
        2: im_dx_pyramid,
        3: im_dy_pyramid,
    }
    for i in range(4):
        for j in range(pyramid_levels):
            plt.subplot(4, pyramid_levels, i * 4 + j + 1)
            plt.imshow(switcher.get(i)[j])
    plt.show()

    print("")

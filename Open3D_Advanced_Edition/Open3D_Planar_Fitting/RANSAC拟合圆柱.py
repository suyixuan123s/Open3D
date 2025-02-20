"""
Author: Yixuan Su
Date: 2025/02/16 16:08
File: RANSAC拟合圆柱.py
Description: 

"""
import numpy as np
import open3d as o3d
import random


# 定义圆柱模型结构体
class CylinderModel:
    def __init__(self):
        self.point = np.array([0, 0, 0])
        self.direction = np.array([0, 0, 1])
        self.r = 0
        self.innerIndices = []


def ransac_fit_cylinder(pcd, sigma, min_r, max_r, iter):
    """
    RANSAC拟合圆柱
    :param pcd: 点云数据文件
    :param sigma: 距离拟合圆柱面两侧的距离
    :param min_r: 设置圆柱最小半径
    :param max_r: 设置圆柱最大半径
    :param iter: 迭代次数
    :return:
    """

    if not pcd.has_normals():
        k = 20  # 计算法向量所用到的K邻域点数
        pcd.estimate_normals(o3d.geometry.KDTreeSearchParamKNN(k))
    # 法向量重定向
    pcd.orient_normals_consistent_tangent_plane(10)
    points = np.asarray(pcd.points)
    normals = np.asarray(pcd.normals)
    nums = points.shape[0]
    range_index = [i for i in range(nums)]  # 获取每个点的索引数值
    model = CylinderModel()
    best_inliers = []
    inliers = []

    for i in range(iter):
        idx = random.sample(range_index, 2)
        sample_data = points[idx, :]
        normal_data = normals[idx, :]
        # 拟合圆柱所需两个点的坐标和法向量
        p1 = sample_data[0, :]
        p2 = sample_data[1, :]
        n1 = normal_data[0, :]
        n2 = normal_data[1, :]

        # 计算圆柱系数
        w = n1 + p1 - p2
        a = np.dot(n1, n1)
        b = np.dot(n1, n2)
        c = np.dot(n2, n2)
        d = np.dot(n1, w)
        e = np.dot(n2, w)
        denominator = a * c - b * b
        # Compute the line parameters of the two closest points
        if denominator < 1e-8:
            sc = 0
            tc = d / b if b > c else e / c  # Use the largest denominator
        else:
            sc = (b * e - c * d) / denominator
            tc = (a * e - b * d) / denominator

        line_pt = p1 + n1 + sc * n1  # point_on_axis
        line_dir = p2 + tc * n2 - line_pt  # axis_direction
        line_dir = line_dir / np.linalg.norm(line_dir)  # 轴向归一化
        # 计算点到轴线的距离
        vec = p1 - line_pt
        r = np.linalg.norm(np.cross(vec, line_dir))
        if r > max_r or r < min_r:
            continue

        # 计算所有点到圆柱轴线的距离
        vecC_stakado = np.stack([line_dir] * nums, 0)
        dist_pt = np.cross(vecC_stakado, (points - line_pt))
        dist_pt = np.linalg.norm(dist_pt, axis=1)
        # 点到圆柱面距离符合阈值的点
        pt_id_inliers = np.where(np.abs(dist_pt - r) <= sigma)[0]
        # 统计符合要求的内点数量
        if len(pt_id_inliers) > len(best_inliers):
            best_inliers = pt_id_inliers
            inliers = best_inliers
            model.point = line_pt
            model.direction = line_dir
            model.r = r
            model.innerIndices = pt_id_inliers
        if len(inliers) > 0.99 * nums:
            break

    return model


if __name__ == "__main__":
    pcd = o3d.io.read_point_cloud(r'/suyixuan/Open3D_Advanced_Edition/data/1.pcd')
    o3d.visualization.draw_geometries([pcd], window_name='原始点云')

    sigma = 0.5  # 距离拟合圆柱两侧的距离
    min_r = 6.0  # 设置圆柱最小半径
    max_r = 8.0  # 设置圆柱最大半径
    max_iters = 500
    cmodel = ransac_fit_cylinder(pcd, sigma, min_r, max_r, max_iters)

    print(f"圆柱中心点:{cmodel.point}\n"
          f"圆柱朝向：{cmodel.direction}\n"
          f"圆柱半径：{cmodel.r}")

    inlier_cloud = pcd.select_by_index(cmodel.innerIndices)
    inlier_cloud.paint_uniform_color([1.0, 0, 0])
    o3d.visualization.draw_geometries([inlier_cloud], window_name='圆柱内点点云')

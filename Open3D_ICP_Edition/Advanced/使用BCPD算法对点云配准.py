"""
Author: Yixuan Su
Date: 2025/02/16 17:18
File: 使用BCPD算法对点云配准.py
Description: 

"""
import open3d as o3d
from probreg import bcpd
import time
import copy

# -----------------------------------------读取点云数据--------------------------------------------
source = o3d.io.read_point_cloud('../data/1.pcd')
target = o3d.io.read_point_cloud('../data/2.pcd')
# --------------------------------------下采样以提高算法效率---------------------------------------
source = source.voxel_down_sample(voxel_size=0.01)
target = target.voxel_down_sample(voxel_size=0.01)
# ----------------------------------------点云上色可视化-------------------------------------------
source.paint_uniform_color([0, 0, 1])
target.paint_uniform_color([0, 1, 0])
o3d.visualization.draw_geometries([source, target], window_name="点云初始位置",
                                  width=1024, height=768,
                                  left=50, top=50,
                                  mesh_show_back_face=False)  # 可视化点云初始位置
# ---------------------------------------BCPD算法进行配准------------------------------------------
start = time.time()

tf_param = bcpd.registration_bcpd(source, target)
result = copy.deepcopy(source)
result.points = tf_param.transform(result.points)
print("配准用时： %.3f sec.\n" % (time.time() - start))
# 可视化配准结果
target.paint_uniform_color([1, 0, 0])
result.paint_uniform_color([0, 0, 1])
o3d.visualization.draw_geometries([target, result], window_name="BCPD算法配准",
                                  width=1024, height=768,
                                  left=50, top=50,
                                  mesh_show_back_face=False)


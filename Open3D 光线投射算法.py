"""
Author: Yixuan Su
Date: 2025/02/19 20:16
File: Open3D 光线投射算法.py
Description: 

"""
import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt

# --------------------------------------添加模型---------------------------------------
# 加载mesh模型并转到open3d.t.geometry.TriangleMesh
cube = o3d.geometry.TriangleMesh.create_box().translate([0, 0, 0])
cube = o3d.t.geometry.TriangleMesh.from_legacy(cube)
# 创建一个场景，并添加三角形网格
scene = o3d.t.geometry.RaycastingScene()
cube_id = scene.add_triangles(cube)  # 返回添加的几何图形的ID。这个ID可以用来识别哪个网格被射线击中。
print(cube_id)
# -------------------------------------光线投射----------------------------------------
# 生成射线，射线是带有原点和方向的6D向量。
# 创建两条射线
# 第一条射线的坐标： (0.5,0.5,10) 方向： (0,0,-1).
# 第二条射线的坐标： (-1,-1,-1) 方向： (0,0,-1).
rays = o3d.core.Tensor([[0.5, 0.5, 10, 0, 0, -1], [-1, -1, -1, 0, 0, -1]],
                       dtype=o3d.core.Dtype.Float32)

ans = scene.cast_rays(rays)
# 结果包含与场景中的几何图形可能相交的信息。
print(ans.keys())
print(ans['t_hit'].numpy(), ans['geometry_ids'].numpy())

# ------------------------------创建一个有多个对象的场景----------------------------------
cube = o3d.geometry.TriangleMesh.create_box().translate([0, 0, 0])
cube = o3d.t.geometry.TriangleMesh.from_legacy(cube)
torus = o3d.geometry.TriangleMesh.create_torus().translate([0, 0, 2])
torus = o3d.t.geometry.TriangleMesh.from_legacy(torus)
sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.5).translate(
    [1, 2, 3])
sphere = o3d.t.geometry.TriangleMesh.from_legacy(sphere)

scene = o3d.t.geometry.RaycastingScene()
scene.add_triangles(cube)
scene.add_triangles(torus)
_ = scene.add_triangles(sphere)

rays = o3d.t.geometry.RaycastingScene.create_rays_pinhole(
    fov_deg=90,
    center=[0, 0, 2],
    eye=[2, 3, 0],
    up=[0, 1, 0],
    width_px=640,
    height_px=480,
)

ans = scene.cast_rays(rays)
# 输出张量保持射线的形状，我们可以直接用matplotlib可视化击中的距离来得到深度图。
# --------------------------------------可视化结果-------------------------------------
plt.figure()
# 默认不支持中文，需要配置RC参数
plt.rcParams['font.sans-serif'] = 'SimHei'
# 默认不支持负号，需要配置RC参数
plt.rcParams['axes.unicode_minus'] = False
plt.imshow(ans['t_hit'].numpy())
plt.title("深度图")
plt.show()
# 进一步，我们可以绘制其他结果可视化原始法线，..
# use abs to avoid negative values
plt.figure()
plt.imshow(np.abs(ans['primitive_normals'].numpy()))
plt.title("法线")
plt.show()
plt.figure()
plt.imshow(ans['geometry_ids'].numpy(), vmax=3)
plt.title("被击中的面片索引")
plt.show()

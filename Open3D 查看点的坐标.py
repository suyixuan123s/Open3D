"""
Author: Yixuan Su
Date: 2025/02/18 15:26
File: Open3D 查看点的坐标.py
Description: 

"""
import numpy as np
import open3d as o3d


def pick_points(cloud):
    print("   Press [shift + right click] to undo point picking")
    print(" After picking points, press 'Q' to close the window")

    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window(window_name='查看点坐标', width=800, height=800, left=50, top=50, visible=True)
    vis.add_geometry(cloud)



    vis.run()  # user picks points
    vis.destroy_window()
    picked_indices = vis.get_picked_points()  # 获取选中的点的索引
    return vis.get_picked_points()


if __name__ == "__main__":
    pcd = o3d.io.read_point_cloud("../data/飞机1.pcd")
    # 调用 pick_points 函数选择点
    picked_indices = pick_points(pcd)

    # 获取点云中所有点的坐标
    points = np.asarray(pcd.points)

    # 输出选中点的坐标
    print("选中点的坐标:")
    for idx in picked_indices:
        print(f"点 {idx} 的坐标: {points[idx]}")



"""
Author: Yixuan Su
Date: 2025/02/11 13:02
File: Open3D_Basic_Edition 读取、显示、保存图片【2025最新版】.py
Description: 

"""
import open3d as o3d

print("Testing IO for images")
img = o3d.io.read_image("../../data/OIP (1).jpg")  # 读取图片（支持jpg和png格式）
print(img)  # 图片大小
o3d.io.write_image("../../data/天使.jpg", img)  # 保存图片
o3d.visualization.draw_geometries([img], window_name="Open3D显示图像",
                                  width=1024, height=768,
                                  left=50, top=50,
                                  mesh_show_back_face=False)  # 显示图片

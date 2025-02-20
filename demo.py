"""
Author: Yixuan Su
Date: 2025/02/19 15:21
File: demo.py
Description: 

"""
import open3d as o3d
print(o3d.__version__)
try:
    from open3d.visualization import gui
except ImportError as e:
    print(f"ImportError: {e}")
else:
    print("Successfully imported 'open3d.visualization.gui'")

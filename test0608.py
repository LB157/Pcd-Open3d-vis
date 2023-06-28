import os
import time

import numpy as np
import open3d as o3d
from matplotlib import pyplot as plt

folder_path = '/home/liubo/Downloads/pcd'  # 本人自己的测试数据路径，直接替换即可

def get_mpl_colormap(cmap_name):
    cmap = plt.get_cmap(cmap_name)
    # Initialize the matplotlib color map
    sm = plt.cm.ScalarMappable(cmap=cmap)
    # Obtain linear color range
    x = np.linspace(0, 1, 256)
    x1 = np.linspace(0, 0.2, 50)
    x2 = np.linspace(0.2, 0.6, 100)
    x3 = np.linspace(0.6, 1, 106)
    y = np.concatenate([x1, x2, x3], 0)
    color_range = sm.to_rgba(y, bytes=True)[:, 2::-1]
    return color_range.reshape(256, 3).astype(np.float32) / 255.0

def load_pcd_to_ndarray(pcd_path):
    with open(pcd_path) as f:
        while True:
            ln = f.readline().strip()
            if ln.startswith('DATA'):
                break

        points = np.loadtxt(f)
        points = points[:, 0:4]
        return points



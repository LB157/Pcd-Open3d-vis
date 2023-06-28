
# kitti  pcd点云通过读取view_point.jsion实现可视化

import os
import time

import numpy as np
import open3d as o3d
from matplotlib import pyplot as plt

# folder_path = '/home/liubo/Downloads/PCD_POINTS_SENSOR'  # 本人自己的测试数据路径，直接替换即可
folder_path = '/home/liubo/Downloads/dataimport/output/kitti/pcd'  # 本人自己的测试数据路径，直接替换即可


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



files = []
for root,dirs,file in os.walk(folder_path):
    for name in file:
        if ".DS_Store" not in name:
            files.append(name)

files.sort(key = lambda x:(os.path.splitext(x)))


vis = o3d.visualization.Visualizer()
vis.create_window(width=1853, height=1025)  # 创建窗口
pcd = o3d.geometry.PointCloud()
render_option: o3d.visualization.RenderOption = vis.get_render_option()	#设置点云渲染参数
render_option.background_color = np.array([0, 0, 0])	#设置背景色（这里为黑色）
render_option.point_size = 2.0	#设置渲染点的大小
to_reset = True
vis.add_geometry(pcd)

ctr = vis.get_view_control()#获取视角控制
parameters = o3d.io.read_pinhole_camera_parameters("viewpoint.json")



while 1:
    for file_index in range(len(files)):
        print("Field of view (before changing) %.2f" % ctr.get_field_of_view())
        data_path = os.path.join(folder_path, files[file_index])
        point_cloud = load_pcd_to_ndarray(data_path)

        point_xyz = point_cloud[:, :3]  # x, y, z
        print(len(point_xyz))
        point_intensity = point_cloud[:, 3]  # intensity

        pcd.points = o3d.utility.Vector3dVector(point_xyz)

        range_data = np.copy(point_intensity)
        viridis_range = 255 - ((range_data - range_data.min()) /
                         (range_data.max() - range_data.min()) *
                         255).astype(np.uint8)

        viridis_map = get_mpl_colormap("coolwarm")

        viridis_colors = viridis_map[viridis_range]


        pcd.colors = o3d.utility.Vector3dVector(viridis_colors)
        vis.update_geometry(pcd)

        print("index= %.2f" % file_index)
        #time.sleep(0.5)
        time.sleep(0.1)
     
        if to_reset:
            vis.reset_view_point(True)
            to_reset = False
        ctr.convert_from_pinhole_camera_parameters(parameters)


        vis.poll_events()
        vis.update_renderer()

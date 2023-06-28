#!/usr/bin/python3
# -*- coding: utf-8 -*-

# open3ddd实现pcd点云固定视角可视化

import os
import open3d as o3d
import numpy as np
import time
from matplotlib import pyplot as plt

folder_path = '/home/liubo/Downloads/PCD_POINTS_SENSOR'  # 本人自己的测试数据路径，直接替换即可
to_reset = True

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



def save_view_point(pcd_numpy, filename):
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    pcd = o3d.open3d.geometry.PointCloud()
    pcd.points = o3d.open3d.utility.Vector3dVector(pcd_numpy)
    vis.add_geometry(pcd)
    axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1, origin=[0, 0, 0])
    vis.add_geometry(axis)
    vis.run()  # user changes the view and press "q" to terminate
    param = vis.get_view_control().convert_to_pinhole_camera_parameters()
    o3d.io.write_pinhole_camera_parameters(filename, param)
    vis.destroy_window()


def draw_color(color_h, color_l, pcd):
    color_h = np.array(color_h, np.float32).reshape(1, 3)
    color_l = np.array(color_l, np.float32).reshape(1, 3)

    raw_points = np.array(pcd.points).copy()
    hight = raw_points[:, 2:]
    hight = np.clip(hight, -3, 1)
    colors = color_l + (hight - (-3)) * (color_h - color_l) / 4.0
    pcd.colors = o3d.utility.Vector3dVector(colors)
    return pcd


def vis_cons(files_dir, vis_detect_result=False):
    files = os.listdir(files_dir)
    files.sort(key = lambda x:int(x.split('_')[3]))
    
    pcds = []
    for f in files:
        pcd_path = os.path.join(files_dir, f)
        pcd = o3d.open3d.geometry.PointCloud()  # 创建点云对象
        point_cloud = load_pcd_to_ndarray(pcd_path)
        point_xyz = point_cloud[:, :3]  # x, y, z
        point_intensity = point_cloud[:, 3]  # intensity

        #raw_point = np.fromfile(pcd_path, dtype=np.float32).reshape(-1, 4)[:, :3]
        pcd.points = o3d.open3d.utility.Vector3dVector(point_xyz)  # 将点云数据转换为Open3d可以直接使用的数据类型
        range_data = np.copy(point_intensity)
        viridis_range = 255 - ((range_data - range_data.min()) /
                               (range_data.max() - range_data.min()) *
                               255).astype(np.uint8)
        viridis_map = get_mpl_colormap("coolwarm")
        viridis_colors = viridis_map[viridis_range]
        pcd.colors = o3d.utility.Vector3dVector(viridis_colors)
        #pcd = draw_color([1.0, 0.36, 0.2], [1.0, 0.96, 0.2], pcd)
        pcds.append(pcd)
    # if vis_detect_result:
    #    batch_results = np.load('batch_results.npy',allow_pickle=True)

    vis = o3d.visualization.Visualizer()
    vis.create_window()
    opt = vis.get_render_option()
    opt.background_color = np.asarray([0, 0, 0])
    opt.point_size = 2
    opt.show_coordinate_frame = False
    ctr = vis.get_view_control()
    if os.path.exists("viewpoint.json"):
        #ctr = vis.get_view_control()
        param = o3d.io.read_pinhole_camera_parameters("viewpoint.json")
        ctr.convert_from_pinhole_camera_parameters(param)
    to_reset = True

    for i in range(len(pcds)):
        vis.clear_geometries()
        axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1, origin=[0, 0, 0])
        vis.add_geometry(axis)
        vis.add_geometry(pcds[i])
        #vis.run()
        ctr.convert_from_pinhole_camera_parameters(param)
        #vis.update_geometry(pcds[i])
        if to_reset:
            vis.reset_view_point(True)
            to_reset = False
        ctr.convert_from_pinhole_camera_parameters(param)
        vis.poll_events()
        vis.update_renderer()
        time.sleep(0.5)

        #vis.run()
    vis.destroy_window()


if __name__ == '__main__':
    exp_pcd_file = r"/home/liubo/Downloads/PCD_POINTS_SENSOR/point_cloud_frame_30_time_0.966667_1.pcd"
    # exp_pcd_file = r"/home/liubo/Downloads/4号-20230131162446(2)/dev/shm/1684739465443479_222bf492-f534-402f-92de-2fdb8397e2f6/1646316521876996097/4号-20230131162446/pcd/srcpcd/1675153532.469674.pcd"
    point_cloud = load_pcd_to_ndarray(exp_pcd_file)
   # point_cloud = load_pcd_to_ndarray(exp_pcd_file)

    point_xyz = point_cloud[:, :3]  # x, y, z
    point_intensity = point_cloud[:, 3]  # intensity

    #view_pcd = np.fromfile(os.path.join(exp_pcd_file, '000000.bin'), dtype=np.float32).reshape(-1, 4)[:, :3]
    save_view_point(point_xyz, "viewpoint.json")
    vis_cons(folder_path)

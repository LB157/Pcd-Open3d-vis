#!/usr/bin/python3
# -*- coding: utf-8 -*-

# 标注的semantickitti数据集可视化
# 
# 
import glob
import os
import open3d as o3d
import numpy as np
import time
import yaml
from matplotlib import pyplot as plt

folder_path = '/home/liubo/Downloads/dataimport/output/kitti/pcd'  # 本人自己的测试数据路径，直接替换即可
to_reset = True

def get_mpl_colormap(yaml_name):
    try:
        print("Opening config file %s" %yaml_name )
        CFG = yaml.safe_load(open(yaml_name, 'r'))
    except Exception as e:
        print(e)
        print("Error opening yaml file.")
        quit()
    color_dict = CFG["color_map"]   
    return color_dict

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
    
    file_dir = glob.glob(files_dir + "/*")
    files = []
    for f in file_dir:
        if f.endswith(".DS_Store"):
            pass
        else:
            files.append(f)
    files.sort(key = lambda x:(x.split('/')[-1]))
    label_path=os.path.abspath(os.path.join(files_dir,"../"))
    label_filepath=os.path.join(label_path,'label','')
    yaml_name="semantic-kitti copy.yaml"
    pcds = []
    for f in files:
        pcd_path = f
        pcd = o3d.open3d.geometry.PointCloud()  # 创建点云对象
        point_cloud = load_pcd_to_ndarray(pcd_path)
        point_xyz = point_cloud[:, :3]  # x, y, z
        point_intensity = point_cloud[:, 3]  # intensity
        fname=f.split('/')[-1]
        label_file_path=os.path.join(label_filepath,os.path.splitext(fname)[0]+'.label')
        label=np.fromfile(label_file_path,dtype=np.int32)
        sem_label = label & 0xFFFF  # semantic label in lower half
        sem_label=sem_label.reshape(-1)
        # inst_label = label >> 16    # instance id in upper half
                    
        #raw_point = np.fromfile(pcd_path, dtype=np.float32).reshape(-1, 4)[:, :3]
        pcd.points = o3d.open3d.utility.Vector3dVector(point_xyz)  # 将点云数据转换为Open3d可以直接使用的数据类型
        
        viridis_map = get_mpl_colormap(yaml_name)
        sem_color_lut = np.zeros((len(sem_label), 3), dtype=np.float32)
        for key in range(0,len(sem_label)):
            # sem_color_lut[key] = np.array(value, np.float32) / 255.0
            sem_color_lut[key] = np.array(viridis_map[sem_label[key]],np.float32)/ 255.0
            
        pcd.colors = o3d.utility.Vector3dVector( sem_color_lut)
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
    exp_pcd_file = r"/home/liubo/Downloads/dataimport/output/kitti/pcd/1675153787.820267.pcd"
    # exp_pcd_file = r"/home/liubo/Downloads/4号-20230131162446(2)/dev/shm/1684739465443479_222bf492-f534-402f-92de-2fdb8397e2f6/1646316521876996097/4号-20230131162446/pcd/srcpcd/1675153532.469674.pcd"
    point_cloud = load_pcd_to_ndarray(exp_pcd_file)
   # point_cloud = load_pcd_to_ndarray(exp_pcd_file)

    point_xyz = point_cloud[:, :3]  # x, y, z
    point_intensity = point_cloud[:, 3]  # intensity

    #view_pcd = np.fromfile(os.path.join(exp_pcd_file, '000000.bin'), dtype=np.float32).reshape(-1, 4)[:, :3]
    save_view_point(point_xyz, "viewpoint.json")
    vis_cons(folder_path)




# -*- coding: UTF-8 -*-
import random
import time
import os
import numpy as np
import yaml
import cv2
from TerrainGEN import *
from ModelGEN import *
from CameraProcess import *
from WorldGEN import *
from utils import delete_paging
from GazeboSimulate import GazeboSimulation

if __name__ == "__main__":
    epoch_num = 20
    seed = time.time()
    random.seed(seed)
    yaml_file_name = './config/mars_terrain_params.yaml'
    for epoch in range(epoch_num):
        # 随机获取地形高度图
        
        # 读取配置yaml文件
        param_file = open(yaml_file_name)
        param_data = yaml.load(param_file, Loader=yaml.FullLoader)
        # param_data['terrain_height'] = 0.1
        heightmap_count = param_data['heightmap_count']
        
        heightmap_num = random.randint(1,heightmap_count)
        heightmap_name = 'HM'+str(heightmap_num)+'.png'
        
        # 读取高度图
        heightmap_path = param_data['heightmap_path']
        heightmap = cv2.imread(os.path.join(heightmap_path, heightmap_name), -1)
        
        # 生成地形DEM
        DEM = generate_DEM(heightmap, param_data)
        
        # 生成地形model文件
        seed_terrain = time.time()
        save_path = param_data['terrain_model_save_path']
        length = param_data['terrain_length']
        height = param_data['terrain_height']
        texture_count = param_data['texture_count']
        min_height_list, texture_num_list = generate_terrain_model(heightmap_name, length, height, texture_count=texture_count, save_path=save_path, seed=seed_terrain, param_data=param_data)
        param_data['min_height_list'] = min_height_list
        param_data['texture_nums'] = texture_num_list
        print(texture_num_list)
        
        # 生成地形类别矩阵
        terrain_class_mat = get_terrain_class_mat(param_data, DEM, mode='height')
        
        # 生成地形DTM
        # TODO: 选择典型地形参数随机生成
        DTM = generate_DTM(param_data, DEM, terrain_class_mat)
        DTM_save_path = param_data['DTM_save_path']
        DTM_save_name = param_data['DTM_save_name']
        plugin_config_modify_path = param_data['plugin_config_file']
        save_DTM(DTM, DTM_save_path, DTM_save_name, plugin_config_modify_path)
        
        # 生成岩石分布
        seed_rock = time.time()
        rock_save_path = param_data['rock_model_save_path']
        generate_rocks_model_no_collision(DEM, param_data, terrain_class_mat, seed=seed_rock, save_path=rock_save_path)
        
        # 生成仿真world文件
        world_save_path = param_data['world_save_path']
        generate_Mars_wolrd(save_path=world_save_path)
        
        # 生成相机移动轨迹
        camera_point_num = param_data['camera_point_num']
        camera_pose_list = generate_camera_path(DEM, camera_point_num)
        
        # 删除paging
        delete_paging()
        
        # 启动Gazebo仿真
        launch_file = param_data['launch_file']
        # Gazebo_Simulate = GazeboSimulation(launch_file)
        # Gazebo_Simulate.close()
        Gazebo_Simulate = GazeboSimulation(launch_file)
        
        # 循环设置相机位姿
        Gazebo_Simulate.set_camera_pose(camera_pose_list)
        
        # 关闭Gazebo仿真
        Gazebo_Simulate.close()
        time.sleep(2)
        
        # 更改world文件
        modify_Mars_wolrd(load_path=world_save_path, is_label=True)
        
        # 生成岩石标签model(或许可以修改)
        generate_rocks_model_no_collision(DEM, param_data, terrain_class_mat, seed=seed_rock, save_path=rock_save_path, is_label=True)
        
        # 生成地形标签model(修改)
        length = param_data['terrain_length']
        height = param_data['terrain_height']
        generate_terrain_model(heightmap_name, length, height, texture_count=texture_count, save_path=save_path, seed=seed_terrain, is_label=True, param_data=param_data)
        
        # 删除paging文件
        delete_paging()
        
        # 启动Gazebo仿真
        launch_file = param_data['launch_file']
        Gazebo_Simulate = GazeboSimulation(launch_file)
        
        # 循环设置相机位姿
        Gazebo_Simulate.set_camera_pose(camera_pose_list,is_label=True)
        
        # 关闭Gazebo仿真
        Gazebo_Simulate.close()

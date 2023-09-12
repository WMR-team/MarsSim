#! /usr/bin/env python3
# -*- coding: UTF-8 -*-
import random
import time
import os
import numpy as np
from numpy.random import default_rng
import yaml
import cv2
import sys
sys.path.append('/home/fwh/FWH/MarsSim_v2/src/world_plugins/scripts')
from TerrainGEN import *
from ModelGEN import *
# from CameraProcess import *
from WorldGEN import *
from utils import delete_paging
import pandas as pd


def change_world(seed, use_user_H=False, default_height=1.5, default_rock_dis=None, use_label=False, mode='height', heightmap_num=5, collide_mode='origin', terrain_len=80):
    random.seed(seed)
    yaml_file_name = '/home/fwh/FWH/MarsSim_v2/src/world_plugins/config/mars_terrain_params.yaml'
    
    # 读取配置yaml文件
    param_file = open(yaml_file_name)
    param_data = yaml.load(param_file, Loader=yaml.FullLoader)
    
    # 随机获取地形高度图
    # heightmap_num = random.randint(1,param_data['heightmap_count'])
    heightmap_num = heightmap_num
    heightmap_name = 'HM'+str(heightmap_num)+'.png'
    
    if use_user_H:
        param_data['terrain_height']=default_height
        param_data['terrain_length']=terrain_len
    # 记录随机数据
    return_record={}
    return_record['heightmap_num'] = heightmap_num
    return_record['terrain_height'] = param_data['terrain_height']

    # 读取高度图
    heightmap_path = param_data['heightmap_path']
    heightmap = cv2.imread(os.path.join(heightmap_path, heightmap_name), -1)
    
    # 生成地形DEM
    DEM = generate_DEM(heightmap, param_data)
    
    # 生成地形model文件
    seed_terrain = time.time()
    seed_terrain = seed
    save_path = param_data['terrain_model_save_path']
    length = param_data['terrain_length']
    height = param_data['terrain_height']
    min_height_list, _ = generate_terrain_model(heightmap_name, length, height, save_path=save_path, seed=seed_terrain, texture_count=param_data['texture_count'], return_record=return_record, is_label=use_label, use_whole_tex=use_label, param_data=param_data)
    param_data['min_height_list'] = min_height_list
    param_data['texture_nums'] = return_record['texture_nums']
    
    # 生成地形类别矩阵
    terrain_class_mat = get_terrain_class_mat(param_data, DEM, mode=mode)
    
    # 生成地形DTM
    # TODO: 选择典型地形参数随机生成
    DTM = generate_DTM(param_data, DEM, terrain_class_mat)
    DTM_save_path = param_data['DTM_save_path']
    DTM_save_name = param_data['DTM_save_name']
    plugin_config_modify_path = param_data['plugin_config_file']
    save_DTM(DTM, DTM_save_path, DTM_save_name, plugin_config_modify_path)
    
    # 生成岩石分布
    seed_rock = time.time()
    seed_rock = seed
    rock_save_path = param_data['rock_model_save_path']
    if collide_mode == 'origin':
        rock_list = generate_rocks_model(DEM, param_data, terrain_class_mat, seed=seed_rock, save_path=rock_save_path, rock_dis_rate=default_rock_dis, return_record=return_record, is_label=use_label)
    elif collide_mode == 'cylinder':
        rock_list = generate_rocks_model_cylinder(DEM, param_data, terrain_class_mat, seed=seed_rock, save_path=rock_save_path, rock_dis_rate=default_rock_dis, return_record=return_record, is_label=use_label)
    elif collide_mode == 'no':
        rock_list = generate_rocks_model_no_collision(DEM, param_data, terrain_class_mat, seed=seed_rock, save_path=rock_save_path, rock_dis_rate=default_rock_dis, return_record=return_record, is_label=use_label)
    
    # 生成仿真world文件
    world_save_path = param_data['world_save_path']
    generate_Mars_wolrd(save_path=world_save_path, return_record=return_record)
    modify_Mars_wolrd(load_path=world_save_path, is_label=use_label)
    
    # 删除paging
    delete_paging()
    
    return DEM, rock_list, return_record

if __name__ == "__main__":
    change_world(55, use_user_H=True, default_height=1, default_rock_dis=0.055)
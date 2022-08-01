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

from scripts.ModelGEN import *
sys.path.append('/home/fwh/FWH/MarsSim_v2/src/world_plugins/scripts')
from TerrainGEN import *
from WorldGEN_exp import *
from utils import delete_paging

def change_world(seed, use_user_H=False, default_height=0.1, use_label=False, mode='height', rock_num=3, bedrock_num=3, BN=(-4.4,4.4,-2.4,2.4)):
    random.seed(seed)
    yaml_file_name = '/home/fwh/FWH/MarsSim_v2/src/world_plugins/config/mars_terrain_params.yaml'
    
    # 读取配置yaml文件
    param_file = open(yaml_file_name)
    param_data = yaml.load(param_file, Loader=yaml.FullLoader)
    
    heightmap_num = 4
    heightmap_name = 'HM'+str(heightmap_num)+'.png'
    
    
    if use_user_H:
        param_data['terrain_height']=default_height
    # 记录随机数据
    return_record={}
    return_record['heightmap_num'] = heightmap_num
    return_record['terrain_height'] = param_data['terrain_height']

    # 读取高度图
    heightmap_path = param_data['heightmap_path']
    heightmap = cv2.imread(os.path.join(heightmap_path, heightmap_name), -1)

    heightmap_name2 = 'HM'+str(heightmap_num+1)+'.png'
    heightmap2 = cv2.imread(os.path.join(heightmap_path, heightmap_name2), -1)
    
    # 生成地形DEM
    DEM = generate_DEM(heightmap, param_data)
    DEM2 = generate_DEM(heightmap2, param_data)

    
    # 生成地形model文件
    # return_record={}
    seed_terrain = time.time()
    seed_terrain = seed
    save_path = param_data['terrain_model_save_path']
    length = param_data['terrain_length']
    height = param_data['terrain_height']
    save_path = '/home/fwh/FWH/MarsSim_v2/src/rover_gazebo/models/experiment_terrain'
    # generate_terrain_model_exp(length, height, save_path=save_path, param_data=param_data)
    min_height_list, _ = generate_terrain_model_exp(heightmap_name, length, height, save_path=save_path, seed=seed_terrain, texture_count=param_data['texture_count'], return_record=return_record, param_data=param_data)
    param_data['texture_nums'] = return_record['texture_nums']
    param_data['min_height_list'] = min_height_list
    
    # 生成地形类别矩阵
    terrain_class_mat = get_terrain_class_mat(param_data, DEM, mode=mode)
    
    # 生成地形DTM
    # TODO: 选择典型地形参数随机生成
    DTM = generate_DTM(param_data, DEM2, terrain_class_mat)
    DTM_save_path = param_data['DTM_save_path']
    DTM_save_name = param_data['DTM_save_name']
    plugin_config_modify_path = param_data['plugin_config_file']
    save_DTM(DTM, DTM_save_path, DTM_save_name, plugin_config_modify_path)
    
    # 生成岩石分布
    seed_rock = time.time()
    seed_rock = seed
    rock_save_path = param_data['rock_model_save_path']
    # rock_list = generate_rocks_model(DEM, param_data, terrain_class_mat, seed=seed_rock, save_path=rock_save_path, rock_dis_rate=default_rock_dis, return_record=return_record, is_label=use_label)
    # generate_rocks_model()
    
    # 生成仿真world文件
    world_save_path = param_data['world_save_path']
    rock_list = generate_Mars_wolrd(save_path=world_save_path, rock_num=rock_num, bedrock_num=bedrock_num, DEM=DEM, BN=BN)
    
    # 删除paging
    delete_paging()
    
    return DEM, rock_list
    

if __name__ == "__main__":
    change_world(seed=55, rock_num=3, bedrock_num=3, BN=(-4.4,4.4,-2.4,2.4))
    # 修改world文件，model文件
    # 
    # 输入参数: 岩石个数n, 扁石个数m, 水平边界(x,y)
    
    
    
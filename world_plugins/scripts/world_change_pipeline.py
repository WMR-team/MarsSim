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

current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, current_dir)

from workspace_config import get_config_path, get_models_path, get_data_path

from TerrainGEN import *
from ModelGEN import *
# from CameraProcess import *
from WorldGEN import *
from utils import delete_paging
import pandas as pd


def resolve_config_paths(param_data):
    """解析 $(find package)/path 格式为绝对路径"""
    import re
    def _resolve(p):
        if not isinstance(p, str) or '$(find ' not in p:
            return p
        m = re.match(r'\$\(find\s+(\w+)\)(.*)', p)
        if not m:
            return p
        pkg, rel = m.groups()
        try:
            from workspace_config import WorkspaceConfig
            return str(WorkspaceConfig.get_package_path(pkg) / rel.lstrip('/'))
        except:
            return p
    for k, v in param_data.items():
        if isinstance(v, str):
            param_data[k] = _resolve(v)
    return param_data


def change_world(seed, use_user_H=False, default_height=1.0, default_rock_dis=None, use_label=False, mode='height', heightmap_num=5, collide_mode='origin', terrain_len=80):
    random.seed(seed)
    yaml_file_name = get_config_path('mars_terrain_params.yaml')

    # 读取配置yaml文件
    param_file = open(yaml_file_name)
    param_data = yaml.load(param_file, Loader=yaml.FullLoader)
    param_data = resolve_config_paths(param_data)
    
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

    # semantic 模式：灰度图 → terrain_class + 贴图按同一套阈值分配
    if mode == 'semantic':
        terrain_classes = param_data['terrain_classes']
        H, W = heightmap.shape[:2]
        vmin, vmax = int(heightmap.min()), int(heightmap.max())
        # 灰度阈值
        grayscale_thresholds = [vmin + (i + 1) * (vmax - vmin) // terrain_classes for i in range(terrain_classes - 1)]
        # 转为高度阈值（与 generate_DEM 一致的缩放：h = pixel * terrain_height / (2^16-1)）
        height_thresholds = [th * height / (2**16 - 1) for th in grayscale_thresholds]
        texture_nums = list(range(1, terrain_classes + 1))
        print(f"[Semantic] grayscale: {vmin}~{vmax}, height thresholds: {[f'{h:.4f}' for h in height_thresholds]}, textures: {texture_nums}")
        min_height_list, _ = generate_terrain_model_with_custom_textures(
            heightmap_name, length, height, save_path=save_path,
            seed=seed_terrain, texture_nums=texture_nums,
            min_height_list=height_thresholds, return_record=return_record,
            is_label=use_label, use_whole_tex=False, param_data=param_data,
            blend_fade_dist=0)
        param_data['texture_nums'] = return_record['texture_nums']
        # 生成 terrain_class.npy
        terrain_class_raw = np.zeros((H, W), dtype=np.int32)
        for i in range(terrain_classes - 1):
            terrain_class_raw[heightmap >= grayscale_thresholds[i]] = i + 1
        terrain_class_pad = np.zeros((H+1, W+1), dtype=np.int32)
        terrain_class_pad[0:-1, 0:-1] = terrain_class_raw
        terrain_class_pad[0:-1, -1] = terrain_class_raw[:, -1]
        terrain_class_pad[-1, 0:-1] = terrain_class_raw[-1, :]
        terrain_class_pad[-1, -1] = terrain_class_raw[-1, -1]
        class_file = param_data['terrain_class_file']
        os.makedirs(os.path.dirname(class_file), exist_ok=True)
        # y轴翻转，对齐Gazebo heightmap渲染坐标系（Gazebo bottom-left vs 图像 top-left）
        terrain_class_pad = terrain_class_pad[::-1, :]
        np.save(class_file, terrain_class_pad)
        print(f"[Semantic] terrain_class saved: {class_file} (shape={terrain_class_pad.shape}, {terrain_classes} types)")
        for i in range(terrain_classes):
            print(f"  Type {i}: {np.sum(terrain_class_pad == i)} pixels")
    else:
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
    change_world(55, use_user_H=True,
                 default_height=0.01,
                 default_rock_dis=0.005,
                 mode='semantic',
                 heightmap_num=1,
                 terrain_len=100)
#! /usr/bin/env python3
# -*- coding: UTF-8 -*-
import random
import time
import os
import yaml
import cv2
from .TerrainGEN import *
from .ModelGEN import *
import hydra

# from CameraProcess import *
from .WorldGEN import *
from .utils import delete_paging
from .utils import omegaconf_to_dict, print_dict
from omegaconf import DictConfig, OmegaConf
from pathlib import Path


def change_world_core(
    param_data,
    seed,
    use_user_H=False,
    default_height=1.5,
    default_rock_dis=None,
    use_label=False,
    mode='height',
    heightmap_num=5,
    collide_mode='origin',
    terrain_len=80,
):

    random.seed(seed)

    # Resolve repo-relative paths at runtime (keep YAML/config values relative)
    repo_root = Path(__file__).resolve().parents[2]  # .../MarsSim

    def _abs(p):
        if p is None:
            return None
        p = str(p)
        if os.path.isabs(p):
            return p
        return str((repo_root / p).resolve())

    # IMPORTANT:
    # - Do resolve filesystem paths (read/write on disk).
    # - Do NOT resolve "in-model" subpaths used in SDF URIs (e.g. heightmap_int8_path).
    for k in [
        "heightmap_path",
        # "heightmap_int8_path",  # keep as-is (used in model:// URI concatenation)
        "terrain_model_save_path",
        "DTM_save_path",
        "rock_model_save_path",
        "world_save_path",
        "plugin_config_file",
        "terrain_data_file",
        "terrain_class_file",
    ]:
        if k in param_data and param_data[k] is not None:
            param_data[k] = _abs(param_data[k])

    # 随机获取地形高度图
    # heightmap_num = random.randint(1,param_data['heightmap_count'])
    heightmap_num = heightmap_num
    heightmap_name = 'HM' + str(heightmap_num) + '.png'

    if use_user_H:
        param_data['terrain_height'] = default_height
        param_data['terrain_length'] = terrain_len
    # 记录随机数据
    return_record = {}
    return_record['heightmap_num'] = heightmap_num
    return_record['terrain_height'] = param_data['terrain_height']

    # 读取高度图
    heightmap_path = param_data["heightmap_path"]
    heightmap = cv2.imread(os.path.join(heightmap_path, heightmap_name), -1)

    # 生成地形DEM
    DEM = generate_DEM(heightmap, param_data)

    # 生成地形model文件
    seed_terrain = time.time()
    seed_terrain = seed
    save_path = param_data["terrain_model_save_path"]
    length = param_data['terrain_length']
    height = param_data['terrain_height']
    min_height_list, _ = generate_terrain_model(
        heightmap_name,
        length,
        height,
        save_path=save_path,
        seed=seed_terrain,
        texture_count=param_data['texture_count'],
        return_record=return_record,
        is_label=use_label,
        use_whole_tex=use_label,
        param_data=param_data,
    )
    param_data['min_height_list'] = min_height_list
    param_data['texture_nums'] = return_record['texture_nums']

    # 生成地形类别矩阵
    terrain_class_mat = get_terrain_class_mat(param_data, DEM, mode=mode)

    # 生成地形DTM
    # TODO: 选择典型地形参数随机生成
    DTM = generate_DTM(param_data, DEM, terrain_class_mat)
    DTM_save_path = param_data["DTM_save_path"]
    DTM_save_name = param_data["DTM_save_name"]
    plugin_config_modify_path = param_data["plugin_config_file"]
    save_DTM(DTM, DTM_save_path, DTM_save_name, plugin_config_modify_path)

    # 生成岩石分布
    seed_rock = time.time()
    seed_rock = seed
    rock_save_path = param_data["rock_model_save_path"]
    if collide_mode == 'origin':
        rock_list = generate_rocks_model(
            DEM,
            param_data,
            terrain_class_mat,
            seed=seed_rock,
            save_path=rock_save_path,
            rock_dis_rate=default_rock_dis,
            return_record=return_record,
            is_label=use_label,
        )
    elif collide_mode == 'cylinder':
        rock_list = generate_rocks_model_cylinder(
            DEM,
            param_data,
            terrain_class_mat,
            seed=seed_rock,
            save_path=rock_save_path,
            rock_dis_rate=default_rock_dis,
            return_record=return_record,
            is_label=use_label,
        )
    elif collide_mode == 'no':
        rock_list = generate_rocks_model_no_collision(
            DEM,
            param_data,
            terrain_class_mat,
            seed=seed_rock,
            save_path=rock_save_path,
            rock_dis_rate=default_rock_dis,
            return_record=return_record,
            is_label=use_label,
        )

    # 生成仿真world文件
    world_save_path = param_data["world_save_path"]
    generate_Mars_wolrd(save_path=world_save_path, return_record=return_record)
    modify_Mars_wolrd(load_path=world_save_path, is_label=use_label)

    # 删除paging
    delete_paging()

    return DEM, rock_list, return_record


@hydra.main(
    version_base="1.1",
    config_name="mars_terrain_params",
    config_path="../config",
)
def hydra_entry(cfg: DictConfig):
    # 从 cfg 中取参数，然后调用核心函数
    seed = cfg.get("seed", 55)
    use_user_H = cfg.get("use_user_H", False)
    default_height = cfg.get("default_height", 1.5)
    default_rock_dis = cfg.get("default_rock_dis", None)
    use_label = cfg.get("use_label", False)
    mode = cfg.get("mode", "height")
    heightmap_num = cfg.get("heightmap_num", 5)
    collide_mode = cfg.get("collide_mode", "origin")
    terrain_len = cfg.get("terrain_len", 80)

    # 先把 cfg 转 dict
    cfg_dict = omegaconf_to_dict(cfg)

    # NOTE: do NOT rewrite to absolute paths here; keep config repo-relative.
    print_dict(cfg_dict)

    return change_world_core(
        cfg_dict,
        seed,
        use_user_H=use_user_H,
        default_height=default_height,
        default_rock_dis=default_rock_dis,
        use_label=use_label,
        mode=mode,
        heightmap_num=heightmap_num,
        collide_mode=collide_mode,
        terrain_len=terrain_len,
    )


if __name__ == "__main__":
    hydra_entry()

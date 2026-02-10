# -*- coding: UTF-8 -*-

import numpy as np
import cv2
import os
from pathlib import Path

# from ruamel import yaml
import yaml
import pandas as pd


def generate_param_tensor(terrain_class_mat, param_data):
    """根据地形类别矩阵生成地面力学参数张量

    params:
        terrain_class_mat: 地形类别矩阵
        param_data: 预设参数字典
    """
    H = terrain_class_mat.shape[0]
    terrain_classes = param_data["terrain_classes"]
    params_num = param_data["parameters_num"]

    terrain_params = np.zeros((params_num, terrain_classes))
    if terrain_classes == 1:
        for t in range(params_num):
            param = param_data["param_" + str(t)]
            if type(param) == "str":
                param = param.split(" ")
                terrain_params[t, :] = [float(i) for i in param]
            else:
                terrain_params[t, :] = param
    else:
        csv_data = pd.read_csv(
            param_data["terrain_data_file"], delimiter=","
        )  # 读取训练数据
        terrain_params[0, :] = list(csv_data.kc[:terrain_classes])
        terrain_params[1, :] = list(csv_data.kphi[:terrain_classes])
        terrain_params[2, :] = list(csv_data.n0[:terrain_classes])
        terrain_params[3, :] = list(csv_data.n1[:terrain_classes])
        terrain_params[4, :] = list(csv_data.c[:terrain_classes])
        terrain_params[5, :] = list(csv_data.phi[:terrain_classes])
        terrain_params[6, :] = list(csv_data.K[:terrain_classes])

    params_map = np.zeros((params_num, H, H))
    for i in range(H):
        for j in range(H):
            params_map[:, i, j] = terrain_params[
                :, int(terrain_class_mat[i, j])
            ]

    return params_map


def generate_DEM(height_map, param_data):
    """生成地形DEM
    params:
        heightmap: 地形高度图
        param_data: 预设参数字典
    return:
        terrain_map: 地形类别矩阵
    """
    terrain_height = param_data["terrain_height"]
    terrain_length = param_data["terrain_length"]
    H, W = height_map.shape

    height_map_new = np.zeros((H + 1, W + 1))
    height_map_new[0:-1, 0:-1] = height_map
    height_map_new[0:-1, -1] = height_map[:, -1]
    height_map_new[-1, 0:-1] = height_map[-1, :]
    height_map_new[-1, -1] = height_map[-1, -1]
    H, W = height_map_new.shape
    height_map = height_map_new

    height_map = (height_map).astype("float")
    height_map = height_map * terrain_height / (2**16 - 1)
    height_map = height_map[::-1, :]
    start_point = -terrain_length / 2
    end_point = terrain_length / 2

    x = np.linspace(start_point, end_point, H)
    x_grid, y_grid = np.meshgrid(x, x)
    terrain_map = np.zeros((3, H, H))

    terrain_map[0, :, :] = x_grid
    terrain_map[1, :, :] = y_grid
    terrain_map[2, :, :] = height_map  # +0.15
    return terrain_map


def get_terrain_class_mat(param_data, DEM, mode=""):
    """计算地形类别矩阵

    params:
        param_data: 预设参数字典
        DEM: 地形DEM
        mode: 计算模式(如果地形类别为1，则不需要；
                    大于1时，'height'为根据地形高度生成矩阵；'semantic'为根据手工设置的语义图计算)
    return:
        terrain_class_mat:
    """
    l = DEM.shape[1]
    heightmap = DEM[2, :, :]
    terrain_classes = param_data["terrain_classes"]
    if terrain_classes == 1:
        terrain_class_mat = np.zeros((l, l))
    elif terrain_classes > 1:
        if mode == "height":
            min_height_list = param_data["min_height_list"]
            texture_nums = param_data["texture_nums"]
            terrain_class_mat = np.ones((l, l)) * ((texture_nums[0] - 1))
            if terrain_classes < 4:
                for tt in range(terrain_classes - 1):
                    terrain_class_mat[heightmap > min_height_list[tt]] = (
                        texture_nums[tt + 1] - 1
                    )
            else:
                terrain_class_mat[heightmap > min_height_list[0]] = (
                    texture_nums[1] - 1
                )
                terrain_class_mat[heightmap > min_height_list[1]] = (
                    texture_nums[2] - 1
                )
                terrain_class_mat[heightmap > min_height_list[2]] = (
                    texture_nums[3] - 1
                )

        if mode == "semantic":
            terrain_class_file = param_data["terrain_class_file"]
            if terrain_class_file.endswith(".npy"):
                terrain_class_mat = np.load(terrain_class_file)
            elif terrain_class_file.endswith(
                ".png"
            ) or terrain_class_file.endswith(".jpg"):
                terrain_class_mat = cv2.imread(terrain_class_file, -1)
            terrain_class_mat = terrain_class_mat[::-1, :]

    return terrain_class_mat


def save_DTM(DTM, save_path, save_name, plugin_config_modify_path):
    """保存地形DTM

    params:
        DTM: 地形DTM
        save_path: 文件保存路径(可相对 MarsSim 根目录)
        save_name: 文件保存名称
        plugin_config_modify_path: 插件 yaml 路径(可相对 MarsSim 根目录)
    """
    repo_root = Path(__file__).resolve().parents[2]  # .../MarsSim

    save_dir = Path(save_path)
    if not save_dir.is_absolute():
        save_dir = (repo_root / save_dir).resolve()
    save_dir.mkdir(parents=True, exist_ok=True)

    dtm_file = (save_dir / save_name).resolve()

    plugin_yaml = Path(plugin_config_modify_path)
    if not plugin_yaml.is_absolute():
        plugin_yaml = (repo_root / plugin_yaml).resolve()

    with open(dtm_file, "w") as f:
        f.write(str(DTM[0, 0, 0]))
        f.write("\n")
        f.write(str(DTM[0, 0, 1] - DTM[0, 0, 0]))
        f.write("\n")
        f.write(str(DTM.shape[1]))
        l = DTM.shape[1]
        f.write("\n")
        for i in range(l):
            for j in range(l):
                for k in range(10):
                    f.write(str(DTM[k, j, i]))
                    f.write("\n")
    with open(plugin_yaml, "r") as f_y:
        content = yaml.load(f_y, Loader=yaml.FullLoader)
        content["x_min"] = float(DTM[0, 0, 0])
        content["x_max"] = float(DTM[0, 0, -1])
        content["y_min"] = float(DTM[0, 0, 0])
        content["y_max"] = float(DTM[0, 0, -1])
        content["x_grids"] = DTM.shape[1]
        content["y_grids"] = DTM.shape[1]

        # store repo-relative path in YAML (no absolute paths)
        rel = os.path.relpath(str(dtm_file), str(repo_root))
        content["TerrainMapFileName"] = rel.replace("\\", "/")

    with open(plugin_yaml, "w") as f_n:
        yaml.dump(content, f_n, default_flow_style=False)


# 生成地形参数文件
def generate_DTM(param_data, DEM, terrain_class_mat):
    """生成用于轮地力学仿真的参数txt文件

    params:
        param_data: 预设参数文件
        DEM: 地形DEM
        terrain_class_mat: 地形类别矩阵
    return:
        DTM: 地形DTM
    """
    terrain_param_tensor = generate_param_tensor(terrain_class_mat, param_data)
    DTM = np.concatenate((DEM, terrain_param_tensor), axis=0)

    return DTM

# -*- coding: UTF-8 -*-
import random
import numpy as np


def generate_camera_path(DEM, points, is_debug=False):
    '''生成相机路径

    params:
        DEM: 地形DEM
        points: 路径点数量
        is_debug: 是否生成路径点txt文件
    returns:
        camera_pose_list: 相机路径点列表
    '''
    H = DEM.shape[1]
    min_l = DEM[0, 0, 0]
    max_l = DEM[0, 0, -1]
    l = max_l - min_l
    step = DEM[0, 0, 1] - DEM[0, 0, 0]

    pitch_range = [0, 70 / 180 * np.pi]
    yaw_range = [0, 2 * np.pi]

    camera_pose_list = []
    margin = 10

    for i in range(points):
        x = round(min_l + margin + random.random() * (l - 2 * margin), 2)
        y = round(min_l + margin + random.random() * (l - 2 * margin), 2)
        point_x = round((x - min_l) / step)
        point_y = round((y - min_l) / step)
        # point_z = round(DEM[2, point_y, point_x]+random.random()*0.8+0.4, 2)
        point_z = round(DEM[2, point_y, point_x] + 1.5, 2)
        # point_z = 1.5

        # pitch = round(pitch_range[1]*random.random(), 2)
        pitch = 0.2 + random.random() * 0.8
        pitch = 0.43 + random.random() * 0.4
        yaw = round(yaw_range[1] * random.random(), 2)
        camera_pose_list.append([x, y, point_z, pitch, yaw])

    if is_debug:
        with open('camera_path.txt', 'w') as f:
            for camera_pose in range(camera_pose_list):
                pose = (
                    str(camera_pose[0])
                    + '\t'
                    + str(camera_pose[1])
                    + '\t'
                    + str(camera_pose[2])
                    + '\t'
                    + str(camera_pose[3])
                    + '\t'
                    + str(camera_pose[4])
                )
                f.write(pose)
                f.write('\n')

    return camera_pose_list


# 标注图像处理
def label_img_process():
    pass


# 可视化原始图像与标注图像
def visuallize_image():
    pass

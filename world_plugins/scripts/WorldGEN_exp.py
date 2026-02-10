# -*- coding: UTF-8 -*-
import random
import time
from lxml import etree as ET
import os
import numpy as np


def generate_Mars_wolrd(
    save_path='', DEM=0, BN=(0, 0, 0, 0), rock_num=0, bedrock_num=0
):
    '''生成仿真world文件

    params:
        save_path: 保存路径
    '''
    sdf = ET.Element("sdf", version="1.5")
    world = ET.SubElement(sdf, "world", name="MarsWorld")
    gravity = ET.SubElement(world, "gravity")
    gravity.text = '0 0 -9.8'
    physics = ET.SubElement(
        world, "physics", name="default_physcis", default="0", type="ode"
    )
    max_step_size = ET.SubElement(physics, "max_step_size")
    max_step_size.text = '0.001'
    real_time_factor = ET.SubElement(physics, "real_time_factor")
    real_time_factor.text = '1'
    real_time_update_rate = ET.SubElement(physics, "real_time_update_rate")
    real_time_update_rate.text = '1000'
    max_contacts = ET.SubElement(physics, "max_contacts")
    max_contacts.text = '10'
    ode = ET.SubElement(physics, 'ode')
    solver = ET.SubElement(ode, 'solver')
    type_ = ET.SubElement(solver, "type")
    type_.text = 'quick'
    iters = ET.SubElement(solver, "iters")
    iters.text = '50'
    sor = ET.SubElement(solver, 'sor')
    sor.text = '1.0'
    use_dynamic_moi_rescaling = ET.SubElement(
        solver, "use_dynamic_moi_rescaling"
    )
    use_dynamic_moi_rescaling.text = 'false'
    scene = ET.SubElement(world, "scene")
    ambient = ET.SubElement(scene, "ambient")
    ambient.text = '0.5 0.5 0.5 1'
    background = ET.SubElement(scene, "background")
    background.text = '0.7 0.7 0.7 1'
    shadows = ET.SubElement(scene, "shadows")
    shadows.text = '1'
    origin_visual = ET.SubElement(scene, "origin_visual")
    origin_visual.text = '0'
    grid = ET.SubElement(scene, 'grid')
    grid.text = '0'
    light = ET.SubElement(world, "light", type="directional", name="sun")
    cast_shadows = ET.SubElement(light, "cast_shadows")
    cast_shadows.text = 'true'
    pose = ET.SubElement(light, "pose")
    pose.text = '0 0 1000 -0.0 0.0 0'
    diffuse = ET.SubElement(light, 'diffuse')
    diffuse.text = '0.5 0.5 0.5 1'
    specular = ET.SubElement(light, "specular")
    specular.text = '0 0 0 1'
    direction = ET.SubElement(light, "direction")
    direction.text = '0.001 0.001 -1'

    include_scene = ET.SubElement(world, "include")
    uri = ET.SubElement(include_scene, "uri")
    uri.text = 'model://exp_scene'

    include_terrain = ET.SubElement(world, "include")
    uri = ET.SubElement(include_terrain, "uri")
    uri.text = 'model://experiment_terrain'

    # random.seed(time.time())
    x_min = BN[0]
    x_max = BN[1]
    y_min = BN[2]
    y_max = BN[3]
    x_l = x_max - x_min
    y_l = y_max - y_min

    step = DEM[0, 0, 1] - DEM[0, 0, 0]
    min_l = DEM[0, 0, 0]
    max_l = DEM[0, 0, -1]
    l = max_l - min_l
    # for i in range(1):
    #     include_bedrock = ET.SubElement(world, "include")
    #     uri = ET.SubElement(include_bedrock, "uri")
    #     uri.text = 'model://bedrock1'
    #     pose = ET.SubElement(include_bedrock, "pose")
    #     px = random.random()*(x_l-0.8)+x_min+0.4
    #     py = random.random()*(y_l-0.8)+y_min+0.4
    #     pz = DEM[2, round((py-l/2)/step), round((px-l/2)/step)]-0.03
    #     yaw = random.random()*2*np.pi
    #     pose_list = str(px)+' '+str(py)+' '+str(pz)+' 0 0 ' +str(yaw)
    #     pose.text = pose_list

    # include_bedrock = ET.SubElement(world, "include")
    # uri = ET.SubElement(include_bedrock, "uri")
    # uri.text = 'model://bedrock1'
    # pose = ET.SubElement(include_bedrock, "pose")
    # px = random.random()*(x_l-0.8)+x_min+0.4
    # py = random.random()*(y_l-0.8)+y_min+0.4
    # pz = DEM[2, round((py-l/2)/step), round((px-l/2)/step)]-0.02
    # yaw = random.random()*2*np.pi
    # pose_list = str(px)+' '+str(py)+' '+str(pz)+' 0 0 ' +str(yaw)
    # pose.text = pose_list

    rock_list = []
    include_rocks = []
    # i_list = [k+1 for k in range(8)]
    # random.shuffle(i_list)
    for i in range(rock_num):
        include_rock = ET.SubElement(world, "include")
        uri = ET.SubElement(include_rock, "uri")
        ii = random.randint(1, 10)
        uri.text = 'model://exp_rock_' + str(ii)
        pose = ET.SubElement(include_rock, "pose")
        px = random.random() * (x_l) + x_min
        py = random.random() * (y_l) + y_min
        if np.sqrt(px * px + py * py) < 0.7:
            px = px + 0.7
        pz = (
            DEM[2, round((py - l / 2) / step), round((px - l / 2) / step)] + 0.1
        )
        roll = random.random() * 2 * np.pi
        pitch = random.random() * 2 * np.pi
        yaw = random.random() * 2 * np.pi
        pose_list = (
            str(px)
            + ' '
            + str(py)
            + ' '
            + str(pz)
            + ' '
            + str(0)
            + ' '
            + str(0)
            + ' '
            + str(yaw)
        )
        pose.text = pose_list
        rock = {}
        rock['x'] = px
        rock['y'] = py
        rock['D'] = 0.3
        rock_list.append(rock)
        include_rocks.append(include_rock)

    for i in range(bedrock_num):
        include_rock = ET.SubElement(world, "include")
        uri = ET.SubElement(include_rock, "uri")
        ii = random.randint(1, 9)
        uri.text = 'model://exp_bedrock_' + str(ii)
        pose = ET.SubElement(include_rock, "pose")
        px = random.random() * (x_l) + x_min
        py = random.random() * (y_l) + y_min
        pz = DEM[2, round((py - l / 2) / step), round((px - l / 2) / step)]
        yaw = random.random() * 2 * np.pi
        pose_list = str(px) + ' ' + str(py) + ' ' + str(pz) + ' 0 0 ' + str(yaw)
        pose.text = pose_list

        include_rocks.append(include_rock)

    # z = DEM[2, round(x/step), round(y/step)]+0.7*D*random.random()-0.4*D
    # # z = DEM[2, round(x/step), round(y/step)]
    # roll = np.random.random()*np.pi*2-np.pi
    # pitch = np.random.random()*np.pi*2-np.pi
    # yaw = np.random.random()*np.pi*2-np.pi
    # pose_list = str(y-l/2)+' '+str(x-l/2)+' '+str(z)+' ' + \
    #     str(roll)+' '+str(pitch)+' '+str(yaw)

    include_camera = ET.SubElement(world, "include")
    uri = ET.SubElement(include_camera, "uri")
    uri.text = 'model://ai_camera'

    tree = ET.ElementTree(sdf)
    save_file = os.path.join(save_path, 'experiment.world')
    tree.write(save_file, pretty_print=True, xml_declaration=True)

    return rock_list

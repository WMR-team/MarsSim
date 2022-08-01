# -*- coding: UTF-8 -*-
import random
from lxml import etree as ET
import os

def get_random_world_param():
    '''生成world文件中的随机参数'''
    background_color_list = [[217, 153, 105], [211, 153, 115], [192, 157, 127]]
    light_brightness_list = [0.2, 0.45, 0.7]
    ambient_list = [[100, 70, 50], [100, 50, 43], [120, 65, 36]]
    light_pose = [-1.2, 1.2]

    background_color = background_color_list[random.randint(0, 2)]
    # background_color = background_color_list[0]
    background_color = background_color_list[2]
    print(background_color)
    background_color_text = [str(round(i/255,2)) for i in background_color]
    background_color_text = ' '.join(background_color_text)
    background_color_text += ' 1'
    # light_brightness = light_brightness_list[random.randint(0, 2)]
    light_brightness = random.random()*0.5+0.2
    # light_brightness = 0.15
    light_brightness = 0.4281432790159745
    light_brightness_text = (str(light_brightness)+' ')*3 + '1'
    ambient = ambient_list[random.randint(0, 2)]
    # ambient = ambient_list[2]
    ambient = ambient_list[0]
    print(ambient)
    ambient_text = [str(round(i/255, 2)) for i in ambient]
    ambient_text = ' '.join(ambient_text)
    ambient_text += ' 1'
    light_pose_x = (light_pose[1]-light_pose[0])*random.random()+light_pose[0]
    light_pose_y = (light_pose[1]-light_pose[0])*random.random()+light_pose[0]
    light_pose_text = '0 0 1000 '+str(light_pose_x)+' '+str(light_pose_y)+' 0'
    light_pose_text = '0 0 1000 0 0 0'

    return background_color_text, light_brightness_text, ambient_text, light_pose_text

def generate_Mars_wolrd(save_path='', return_record={}):
    '''生成仿真world文件
    
    params: 
        save_path: 保存路径
    '''
    background_color_text, light_brightness_text, ambient_text, light_pose_text = get_random_world_param()
    return_record['background_color'] = background_color_text
    return_record['light_brightness'] = light_brightness_text
    return_record['ambient'] = ambient_text
    return_record['light_pose'] = light_pose_text
    sdf = ET.Element("sdf", version="1.5")
    world = ET.SubElement(sdf, "world", name="MarsWorld")
    gravity = ET.SubElement(world, "gravity")
    gravity.text = '0 0 -3.711'
    physics = ET.SubElement(
        world, "physics", name="default_physcis", default="0", type="ode")
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
        solver, "use_dynamic_moi_rescaling")
    use_dynamic_moi_rescaling.text = 'false'
    scene = ET.SubElement(world, "scene")
    ambient = ET.SubElement(scene, "ambient")
    ambient.text = ambient_text
    background = ET.SubElement(scene, "background")
    background.text = background_color_text
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
    pose.text = light_pose_text
    diffuse = ET.SubElement(light, 'diffuse')
    diffuse.text = light_brightness_text
    specular = ET.SubElement(light, "specular")
    specular.text = '0 0 0 1'
    direction = ET.SubElement(light, "direction")
    direction.text = '0.001 0.001 -1'

    include_terrain = ET.SubElement(world, "include")
    uri = ET.SubElement(include_terrain, "uri")
    uri.text = 'model://mars_terrain'

    include_rock = ET.SubElement(world, "include")
    uri = ET.SubElement(include_rock, "uri")
    uri.text = 'model://mars_rocks'

    include_camera = ET.SubElement(world, "include")
    uri = ET.SubElement(include_camera, "uri")
    uri.text = 'model://ai_camera'
    
    # include_obstacle = ET.SubElement(world, "include")
    # uri = ET.SubElement(include_obstacle, "uri")
    # uri.text = 'model://obstacle'

    tree = ET.ElementTree(sdf)
    save_file = os.path.join(save_path, 'MarsWorld.world')
    tree.write(save_file, pretty_print=True, xml_declaration=True)

def modify_Mars_wolrd(load_path='', is_label=False):
    '''修改仿真world文件
    
    params: 
        load_path: 文件加载路径
        is_label: 是否面向标注任务修改
    '''
    Tree = ET.parse(os.path.join(load_path , 'MarsWorld.world') )
    background_color_text, light_brightness_text, ambient_text, light_pose_text = get_random_world_param()

    sdf = Tree.getroot()
    world = sdf.find('world')
    scene = world.find('scene')
    ambient = scene.find('ambient')
    background = scene.find('background')
    shadows = scene.find('shadows')
    if is_label:
        ambient.text = '1 1 1 1'
        background.text = '1 1 1 1'
        shadows.text = 'false'
    else:
        ambient.text = ambient_text
        background.text = background_color_text
        shadows.text = 'true'
    light = world.find('light')
    if is_label:
        world.remove(light)
    else:
        cast_shadows = light.find('cast_shadows')
        pose = light.find('pose')
        diffuse = light.find('diffuse')
        cast_shadows.text = 'true'
        pose.text = light_pose_text
        diffuse.text = light_brightness_text
    
    save_file = os.path.join(load_path, 'MarsWorld.world')
    Tree.write(save_file, pretty_print=True, xml_declaration=True)
    
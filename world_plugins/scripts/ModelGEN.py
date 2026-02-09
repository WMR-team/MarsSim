# -*- coding: UTF-8 -*-
from lxml import etree as ET
import random
from ConfigGEN import generate_config
from RockDistCalc import calculate_rock_distribution
import numpy as np
import os
import cv2

def generate_rocks_model(DEM, param_data, terrain_class_mat, save_path='', seed=1, is_label=False, rock_dis_rate=None, return_record={}):
    '''生成岩石model文件

    param:
        DEM: 地形DEM
        param_data: 参数字典
        terrain_class_mat: 地形类别矩阵
        save_path: model保存路径
        seed: 随机种子
        is_label: 是否生成标签model
    '''

    random.seed(seed)
    np.random.seed(int(seed))
    generate_config('mars_rocks_lbl', save_path=save_path)
    rock_list = calculate_rock_distribution(DEM, param_data, terrain_class_mat, rock_dis_rate, return_record)
    step = DEM[0, 0, 1] - DEM[0, 0, 0]
    min_l = DEM[0, 0, 0]
    max_l = DEM[0, 0, -1]
    l = max_l-min_l
    # 根据岩石二维分布生成岩石model
    model_name = 'mars_rocks_lbl'
    sdf = ET.Element("sdf", version="1.5")
    model = ET.SubElement(sdf, "model", name=model_name)
    static = ET.SubElement(model, "static")
    static.text = "true"
    # 已知岩石的数量
    rock_num = len(rock_list)
    pose_list = []
    scale_list = []
    for i in range(rock_num):
        link = ET.SubElement(model, "link", name="rock_"+str(i))
        # Collision element of the model
        collision = ET.SubElement(link, "collision", name="collision")
        pose_collision = ET.SubElement(collision, "pose")
        x = rock_list[i]['x']
        y = rock_list[i]['y']
        D = rock_list[i]['D']

        z = DEM[2, round(x/step), round(y/step)]+0.7*D*random.random()-0.4*D
        # z = DEM[2, round(x/step), round(y/step)]
        roll = np.random.random()*np.pi*2-np.pi
        pitch = np.random.random()*np.pi*2-np.pi
        yaw = np.random.random()*np.pi*2-np.pi
        pose_list = str(y-l/2)+' '+str(x-l/2)+' '+str(z)+' ' + \
            str(roll)+' '+str(pitch)+' '+str(yaw)
        pose_collision.text = pose_list
        geometry_collision = ET.SubElement(collision, "geometry")
        mesh_geometry = ET.SubElement(geometry_collision, "mesh")
        scale_mesh = ET.SubElement(mesh_geometry, "scale")
        scale = 1.5*(D+np.random.rand()*D/2)
        scale_list = str(scale)+' '+str(scale)+' '+str(scale)
        scale_mesh.text = scale_list
        uri = ET.SubElement(mesh_geometry, "uri")
        rock_choose = random.randint(1,param_data['rock_count'])
        if is_label:
            rock_name = 'model://mars_rocks_lbl/mars_rock_' + \
                str(rock_choose)+'/rock'+str(rock_choose)+'_c.obj'
        else:
            rock_name = 'model://mars_rocks_lbl/mars_rock_' + \
                str(rock_choose)+'/rock'+str(rock_choose)+'.obj'
        uri.text = rock_name
        # Visual element of the model
        visual = ET.SubElement(link, "visual", name="visual")
        pose_visual = ET.SubElement(visual, "pose")
        pose_visual.text = pose_list
        geometry_visual = ET.SubElement(visual, "geometry")
        mesh_geometry = ET.SubElement(geometry_visual, "mesh")
        scale_mesh = ET.SubElement(mesh_geometry, "scale")
        scale_mesh.text = scale_list
        uri = ET.SubElement(mesh_geometry, "uri")
        uri.text = rock_name
    tree = ET.ElementTree(sdf)
    save_file = os.path.join(save_path, 'model.sdf')
    tree.write(save_file, pretty_print=True, xml_declaration=True)
    return rock_list

def generate_rocks_model_cylinder(DEM, param_data, terrain_class_mat, save_path='', seed=1, is_label=False, rock_dis_rate=None, return_record={}):
    '''生成岩石model文件

    param:
        DEM: 地形DEM
        param_data: 参数字典
        terrain_class_mat: 地形类别矩阵
        save_path: model保存路径
        seed: 随机种子
        is_label: 是否生成标签model
    '''

    random.seed(seed)
    np.random.seed(int(seed))
    generate_config('mars_rocks_lbl', save_path=save_path)
    rock_list = calculate_rock_distribution(DEM, param_data, terrain_class_mat, rock_dis_rate, return_record)
    step = DEM[0, 0, 1] - DEM[0, 0, 0]
    min_l = DEM[0, 0, 0]
    max_l = DEM[0, 0, -1]
    l = max_l-min_l
    # 根据岩石二维分布生成岩石model
    model_name = 'mars_rocks_lbl'
    sdf = ET.Element("sdf", version="1.5")
    model = ET.SubElement(sdf, "model", name=model_name)
    static = ET.SubElement(model, "static")
    static.text = "true"
    # 已知岩石的数量
    rock_num = len(rock_list)
    pose_list = []
    scale_list = []
    for i in range(rock_num):
        link = ET.SubElement(model, "link", name="rock_"+str(i))
        # Collision element of the model
        collision = ET.SubElement(link, "collision", name="collision")
        pose_collision = ET.SubElement(collision, "pose")
        x = rock_list[i]['x']
        y = rock_list[i]['y']
        D = rock_list[i]['D']

        z = DEM[2, round(x/step), round(y/step)]+0.7*D*random.random()-0.3*D
        # z = DEM[2, round(x/step), round(y/step)]
        roll = np.random.random()*np.pi*2-np.pi
        pitch = np.random.random()*np.pi*2-np.pi
        yaw = np.random.random()*np.pi*2-np.pi
        pose_list = str(y-l/2)+' '+str(x-l/2)+' '+str(z)+' ' + \
            str(roll)+' '+str(pitch)+' '+str(yaw)
        pose_collision.text = str(y-l/2)+' '+str(x-l/2)+' '+str(z)+' 0 0 0'
        geometry_collision = ET.SubElement(collision, "geometry")
        cylinder_geometry = ET.SubElement(geometry_collision, "cylinder")
        radius = ET.SubElement(cylinder_geometry, "radius")
        scale = 1.5*(D+np.random.rand()*D/2)
        scale_list = str(scale)+' '+str(scale)+' '+str(scale)
        radius.text = str(1.1*D)

        length = ET.SubElement(cylinder_geometry, "length")
        rock_choose = random.randint(1,param_data['rock_count'])
        if is_label:
            rock_name = 'model://mars_rocks_lbl/mars_rock_' + \
                str(rock_choose)+'/rock'+str(rock_choose)+'_c.obj'
        else:
            rock_name = 'model://mars_rocks_lbl/mars_rock_' + \
                str(rock_choose)+'/rock'+str(rock_choose)+'.obj'
        length.text = str(3*D)
        # Visual element of the model
        visual = ET.SubElement(link, "visual", name="visual")
        pose_visual = ET.SubElement(visual, "pose")
        pose_visual.text = pose_list
        geometry_visual = ET.SubElement(visual, "geometry")
        mesh_geometry = ET.SubElement(geometry_visual, "mesh")
        scale_mesh = ET.SubElement(mesh_geometry, "scale")
        scale_mesh.text = scale_list
        uri = ET.SubElement(mesh_geometry, "uri")
        uri.text = rock_name
    tree = ET.ElementTree(sdf)
    save_file = os.path.join(save_path, 'model.sdf')
    tree.write(save_file, pretty_print=True, xml_declaration=True)
    return rock_list

def generate_rocks_model_no_collision(DEM, param_data, terrain_class_mat, save_path='', seed=1, is_label=False, rock_dis_rate=None, return_record={}):
    '''生成岩石model文件

    param:
        DEM: 地形DEM
        param_data: 参数字典
        terrain_class_mat: 地形类别矩阵
        save_path: model保存路径
        seed: 随机种子
        is_label: 是否生成标签model
    '''

    random.seed(seed)
    np.random.seed(int(seed))
    generate_config('mars_rocks_lbl', save_path=save_path)
    rock_list = calculate_rock_distribution(DEM, param_data, terrain_class_mat, rock_dis_rate, return_record)
    step = DEM[0, 0, 1] - DEM[0, 0, 0]
    min_l = DEM[0, 0, 0]
    max_l = DEM[0, 0, -1]
    l = max_l-min_l
    # 根据岩石二维分布生成岩石model
    model_name = 'mars_rocks_lbl'
    sdf = ET.Element("sdf", version="1.5")
    model = ET.SubElement(sdf, "model", name=model_name)
    static = ET.SubElement(model, "static")
    static.text = "true"
    # 已知岩石的数量
    rock_num = len(rock_list)
    pose_list = []
    scale_list = []
    for i in range(rock_num):
        link = ET.SubElement(model, "link", name="rock_"+str(i))
        # Collision element of the model
        x = rock_list[i]['x']
        y = rock_list[i]['y']
        D = rock_list[i]['D']

        z = DEM[2, round(x/step), round(y/step)]+0.7*D*random.random()-0.3*D
        # z = DEM[2, round(x/step), round(y/step)]
        roll = np.random.random()*np.pi*2-np.pi
        pitch = np.random.random()*np.pi*2-np.pi
        yaw = np.random.random()*np.pi*2-np.pi
        pose_list = str(y-l/2)+' '+str(x-l/2)+' '+str(z)+' ' + \
            str(roll)+' '+str(pitch)+' '+str(yaw)

        scale = 1.5*(D+np.random.rand()*D/2)
        scale_list = str(scale)+' '+str(scale)+' '+str(scale)

        rock_choose = random.randint(1,param_data['rock_count'])
        if is_label:
            rock_name = 'model://mars_rocks_lbl/mars_rock_' + \
                str(rock_choose)+'/rock'+str(rock_choose)+'_c.obj'
        else:
            rock_name = 'model://mars_rocks_lbl/mars_rock_' + \
                str(rock_choose)+'/rock'+str(rock_choose)+'.obj'
        # Visual element of the model
        visual = ET.SubElement(link, "visual", name="visual")
        pose_visual = ET.SubElement(visual, "pose")
        pose_visual.text = pose_list
        geometry_visual = ET.SubElement(visual, "geometry")
        mesh_geometry = ET.SubElement(geometry_visual, "mesh")
        scale_mesh = ET.SubElement(mesh_geometry, "scale")
        scale_mesh.text = scale_list
        uri = ET.SubElement(mesh_geometry, "uri")
        uri.text = rock_name
    tree = ET.ElementTree(sdf)
    save_file = os.path.join(save_path, 'model.sdf')
    tree.write(save_file, pretty_print=True, xml_declaration=True)
    return rock_list


# 生成地形model文件
def generate_terrain_model(heightmap_name, length, height, save_path='', seed=1, is_label=False, texture_count=26, return_record={}, use_whole_tex=False, param_data={}):
    '''生成地形model文件

    param:
        heightmap_name: 地形高度图文件名
        save_path: model保存路径
        seed: 随机种子
        is_label: 是否生成标签model
        texture_count: 地形纹理数量
    return:
        min_height_list: 多地形过渡高度列表
    '''

    random.seed(seed)
    # texture_num1 = random.randint(1, texture_count)
    # texture_num2 = random.randint(1, texture_count)
    # texture_num3 = random.randint(1, texture_count)
    # texture_num4 = random.randint(1, texture_count)

    ii = [i+1 for i in range(texture_count)]
    random.shuffle(ii)
    texture_num1 = ii[0]
    texture_num2 = ii[1]
    texture_num3 = ii[2]
    texture_num4 = ii[3]

    min_height_list = [height/5, height/2, height/5*4]
    if texture_count == 4:
        ii = [i+1 for i in range(3)]
        random.shuffle(ii)
        texture_num1 = 4
        texture_num2 = ii[0]
        texture_num3 = ii[1]
        texture_num4 = ii[2]
        min_height_list = []
        heightmap_path = param_data['heightmap_path']
        heightmap = cv2.imread(os.path.join(heightmap_path, heightmap_name), -1)
        heightmap = heightmap//256
        h,w = heightmap.shape
        s = 0
        hs = 1
        for hh in range(0,255):
            s += np.sum(heightmap==hh)
            if s >= h*w*hs/4:
                min_height_list.append(hh/255*height)
                hs += 1
    min_height_list[0] = min_height_list[0]*0.75
    print(min_height_list)

    heightmap = cv2.imread(os.path.join(heightmap_path, heightmap_name), -1)
    heightmap = heightmap//256
    heightmap = heightmap/255.0*height

    sem_img = np.ones_like(heightmap)*texture_num4
    sem_img[heightmap<min_height_list[2]]=texture_num3
    sem_img[heightmap<min_height_list[1]]=texture_num2
    sem_img[heightmap<min_height_list[0]]=texture_num1
    sem_img = sem_img.astype('uint8')

    # colormap = label_colormap(76)
    colormap = np.array([[0,0,0],[150,0,0],[197,255,49],[0,180,255],[0,0,150]])
    colormap = (colormap).astype(np.uint8)
    lbl_viz = colormap[sem_img]
    lbl_viz = lbl_viz.astype(np.uint8)
    # lbl_viz = lbl_viz[:,:,::-1]
    cv2.imwrite('/home/tipriest/Documents/MarsSim_v2_ws/src/MarsSim/rover_gazebo/models/mars_terrain/whole_tex/class.png',lbl_viz)

    return_record['min_heights'] = min_height_list

    texture_num_list = [texture_num1, texture_num2, texture_num3, texture_num4]
    return_record['texture_nums'] = texture_num_list
    # 生成config文件
    generate_config('mars_terrain', save_path=save_path)

    sdf = ET.Element("sdf", version="1.6")
    model = ET.SubElement(sdf, "model", name='mars_terrain')
    model_pose = ET.SubElement(model, "pose")
    model_pose.text = '0 0 0 0 0 0'
    static = ET.SubElement(model, "static")
    static.text = 'true'
    link = ET.SubElement(model, "link", name='link')

    visual = ET.SubElement(link, "visual", name="visual")
    geometry = ET.SubElement(visual, 'geometry')
    heightmap = ET.SubElement(geometry, 'heightmap')
    if use_whole_tex:
        import plot_geometry
        plot_geometry.generate_class_mat()
        texture = ET.SubElement(heightmap, 'texture')
        diffuse = ET.SubElement(texture, 'diffuse')
        normal = ET.SubElement(texture, 'normal')
        diffuse.text = 'model://mars_terrain/whole_tex/terrain_tex.png'
        # diffuse.text = 'model://mars_terrain/simulation_label/'+'2_color.jpg'
        normal.text = 'model://mars_terrain/simulation_label/'+'NRM.png'
        size = ET.SubElement(texture, 'size')
        size.text = str(length)

    else:
        for i in range(4):
            texture = ET.SubElement(heightmap, 'texture')
            diffuse = ET.SubElement(texture, 'diffuse')
            normal = ET.SubElement(texture, 'normal')
            texture_path = param_data['texture_path']
            if is_label:
                diffuse.text = 'model://mars_terrain/simulation_label/' + \
                    str((texture_num_list[i]-1)//3+1)+'_color.jpg'
                    # '8_color.jpg'

                # diffuse.text = 'model://mars_terrain/simulation_label/'+'2_color.jpg'
                normal.text = 'model://mars_terrain/simulation_label/'+'NRM.png'
            else:
                diffuse.text = 'model://mars_terrain/'+ texture_path + \
                    str(texture_num_list[i])+'.jpg'
                normal.text = 'model://mars_terrain/'+ texture_path + \
                    str(texture_num_list[i])+'_NRM.png'
            size = ET.SubElement(texture, 'size')
            size.text = '1'

        for i in range(3):
            blend = ET.SubElement(heightmap, 'blend')
            min_height = ET.SubElement(blend, 'min_height')
            min_height.text = str(min_height_list[i])
            fade_dist = ET.SubElement(blend, 'fade_dist')
            if is_label:
                fade_dist.text = '0'
            else:
                fade_dist.text = '0.02'

    uri = ET.SubElement(heightmap, 'uri')
    heightmap_int8_path = param_data['heightmap_int8_path']
    uri.text = 'model://mars_terrain/'+heightmap_int8_path+heightmap_name
    # uri.text = 'model://mars_terrain/heightmaps_int8/'+heightmap_name
    size = ET.SubElement(heightmap, 'size')
    terrain_size = str(length)+' '+str(length)+' '+str(height)
    size.text = terrain_size
    pos = ET.SubElement(heightmap, 'pos')
    pos.text = '0 0 0'

    tree = ET.ElementTree(sdf)
    save_file = os.path.join(save_path, 'model.sdf')
    tree.write(save_file, pretty_print=True, xml_declaration=True)

    return min_height_list, texture_num_list
def generate_terrain_model_exp(heightmap_name, length, height, save_path='', seed=1, texture_count=26, return_record={}, param_data={}):
    random.seed(seed)
    # texture_num1 = random.randint(1, texture_count)
    # texture_num2 = random.randint(1, texture_count)
    # texture_num3 = random.randint(1, texture_count)
    # texture_num4 = random.randint(1, texture_count)

    ii = [i+1 for i in range(texture_count)]
    random.shuffle(ii)
    texture_num1 = 1
    texture_num2 = 1
    texture_num3 = 1
    texture_num4 = 1

    min_height_list = [height/5, height/2, height/5*4]
    if param_data['terrain_classes'] == 4:
        ii = [i+1 for i in range(3)]
        random.shuffle(ii)
        texture_num1 = 1
        texture_num2 = 1
        texture_num3 = 1
        texture_num4 = 2
        min_height_list = []
        heightmap_path = param_data['heightmap_path']
        heightmap = cv2.imread(os.path.join(heightmap_path, heightmap_name), -1)
        heightmap = heightmap//256
        h,w = heightmap.shape
        s = 0
        hs = 1
        for hh in range(0,255):
            s += np.sum(heightmap==hh)
            if s >= h*w*hs/4:
                min_height_list.append(hh/255*height)
                hs += 1
    min_height_list[-1] = height*0.91
    print(min_height_list)

    return_record['min_heights'] = min_height_list

    texture_num_list = [texture_num1, texture_num2, texture_num3, texture_num4]
    return_record['texture_nums'] = texture_num_list
    # 生成config文件
    generate_config('mars_terrain', save_path=save_path)

    sdf = ET.Element("sdf", version="1.6")
    model = ET.SubElement(sdf, "model", name='mars_terrain')
    model_pose = ET.SubElement(model, "pose")
    model_pose.text = '0 0 0 0 0 0'
    static = ET.SubElement(model, "static")
    static.text = 'true'
    link = ET.SubElement(model, "link", name='link')

    visual = ET.SubElement(link, "visual", name="visual")
    geometry = ET.SubElement(visual, 'geometry')
    heightmap = ET.SubElement(geometry, 'heightmap')
    for i in range(4):
        texture = ET.SubElement(heightmap, 'texture')
        diffuse = ET.SubElement(texture, 'diffuse')
        normal = ET.SubElement(texture, 'normal')
        texture_path = param_data['texture_path']
        diffuse.text = 'model://mars_terrain/'+ texture_path + \
            str(texture_num_list[i])+'.jpg'
        normal.text = 'model://mars_terrain/'+ texture_path + \
            str(texture_num_list[i])+'_NRM.png'
        size = ET.SubElement(texture, 'size')
        size.text = '1'
        if i==3:
            size.text = '0.8'

    for i in range(3):
        blend = ET.SubElement(heightmap, 'blend')
        min_height = ET.SubElement(blend, 'min_height')
        min_height.text = str(min_height_list[i])
        fade_dist = ET.SubElement(blend, 'fade_dist')
        fade_dist.text = '0.001'

    uri = ET.SubElement(heightmap, 'uri')
    heightmap_int8_path = param_data['heightmap_int8_path']
    uri.text = 'model://mars_terrain/'+heightmap_int8_path+heightmap_name
    # uri.text = 'model://mars_terrain/heightmaps_int8/'+heightmap_name
    size = ET.SubElement(heightmap, 'size')
    terrain_size = str(length)+' '+str(length)+' '+str(height)
    size.text = terrain_size
    pos = ET.SubElement(heightmap, 'pos')
    pos.text = '0 0 0'

    tree = ET.ElementTree(sdf)
    save_file = os.path.join(save_path, 'model.sdf')
    tree.write(save_file, pretty_print=True, xml_declaration=True)

    return min_height_list, texture_num_list

def generate_flat_terrain_model(heightmap_name, length, height, save_path='', seed=1, texture_count=26, param_data={}):

    # 生成config文件
    generate_config('mars_terrain', save_path=save_path)

    sdf = ET.Element("sdf", version="1.6")
    model = ET.SubElement(sdf, "model", name='mars_terrain')
    model_pose = ET.SubElement(model, "pose")
    model_pose.text = '0 0 0 0 0 0'
    static = ET.SubElement(model, "static")
    static.text = 'true'
    link = ET.SubElement(model, "link", name='link')

    visual = ET.SubElement(link, "visual", name="visual")
    geometry = ET.SubElement(visual, 'geometry')
    heightmap = ET.SubElement(geometry, 'heightmap')

    texture = ET.SubElement(heightmap, 'texture')
    diffuse = ET.SubElement(texture, 'diffuse')
    normal = ET.SubElement(texture, 'normal')
    texture_path = param_data['texture_path']
    diffuse.text = 'model://mars_terrain/'+ texture_path + '4.jpg'
    normal.text = 'model://mars_terrain/'+ texture_path + '4_NRM.png'
    size = ET.SubElement(texture, 'size')
    size.text = '1'


    uri = ET.SubElement(heightmap, 'uri')
    heightmap_int8_path = param_data['heightmap_int8_path']
    uri.text = 'model://mars_terrain/'+heightmap_int8_path+heightmap_name
    # uri.text = 'model://mars_terrain/heightmaps_int8/'+heightmap_name
    size = ET.SubElement(heightmap, 'size')
    terrain_size = str(length)+' '+str(length)+' '+str(height)
    size.text = terrain_size
    pos = ET.SubElement(heightmap, 'pos')
    pos.text = '0 0 0'

    tree = ET.ElementTree(sdf)
    save_file = os.path.join(save_path, 'model.sdf')
    tree.write(save_file, pretty_print=True, xml_declaration=True)

# def generate_terrain_model_exp(heightmap_name, length, height, save_path='', seed=1, is_label=False, texture_count=26, return_record={}, use_whole_tex=False, param_data={}):
#     '''生成地形model文件

#     param:
#         heightmap_name: 地形高度图文件名
#         save_path: model保存路径
#         seed: 随机种子
#         is_label: 是否生成标签model
#         texture_count: 地形纹理数量
#     return:
#         min_height_list: 多地形过渡高度列表
#     '''

#     texture_num1 = 1
#     texture_num2 = 2

#     min_height_list = [height/10]


#     return_record['min_heights'] = min_height_list

#     texture_num_list = [texture_num1, texture_num2]
#     return_record['texture_nums'] = texture_num_list
#     # 生成config文件
#     generate_config('mars_terrain', save_path=save_path)

#     sdf = ET.Element("sdf", version="1.6")
#     model = ET.SubElement(sdf, "model", name='mars_terrain')
#     model_pose = ET.SubElement(model, "pose")
#     model_pose.text = '0 0 0 0 0 0'
#     static = ET.SubElement(model, "static")
#     static.text = 'true'
#     link = ET.SubElement(model, "link", name='link')

#     visual = ET.SubElement(link, "visual", name="visual")
#     geometry = ET.SubElement(visual, 'geometry')
#     heightmap = ET.SubElement(geometry, 'heightmap')

#     for i in range(2):
#         texture = ET.SubElement(heightmap, 'texture')
#         diffuse = ET.SubElement(texture, 'diffuse')
#         normal = ET.SubElement(texture, 'normal')
#         texture_path = param_data['texture_path']
#         diffuse.text = 'model://mars_terrain/'+ texture_path + \
#             str(texture_num_list[i])+'.jpg'
#         normal.text = 'model://mars_terrain/'+ texture_path + \
#             str(texture_num_list[i])+'_NRM.png'
#         size = ET.SubElement(texture, 'size')
#         size.text = '0.5'
#         # if i==1:
#         #     size.text = '0.5'

#     blend = ET.SubElement(heightmap, 'blend')
#     min_height = ET.SubElement(blend, 'min_height')
#     min_height.text = str(min_height_list[0])
#     fade_dist = ET.SubElement(blend, 'fade_dist')
#     fade_dist.text = '0.01'

#     uri = ET.SubElement(heightmap, 'uri')
#     heightmap_int8_path = param_data['heightmap_int8_path']
#     uri.text = 'model://mars_terrain/'+heightmap_int8_path+heightmap_name
#     # uri.text = 'model://mars_terrain/heightmaps_int8/'+heightmap_name
#     size = ET.SubElement(heightmap, 'size')
#     terrain_size = str(length)+' '+str(length)+' '+str(height)
#     size.text = terrain_size
#     pos = ET.SubElement(heightmap, 'pos')
#     pos.text = '0 0 0'

#     tree = ET.ElementTree(sdf)
#     save_file = os.path.join(save_path, 'model.sdf')
#     tree.write(save_file, pretty_print=True, xml_declaration=True)

#     return min_height_list, texture_num_list


def label_colormap(N=256):

    def bitget(byteval, idx):
        return ((byteval & (1 << idx)) != 0)

    cmap = np.zeros((N, 3))
    for i in range(0, N):
        id = i
        r, g, b = 0, 0, 0
        for j in range(0, 8):
            r = np.bitwise_or(r, (bitget(id, 0) << 7 - j))
            g = np.bitwise_or(g, (bitget(id, 1) << 7 - j))
            b = np.bitwise_or(b, (bitget(id, 2) << 7 - j))
            id = (id >> 3)
        cmap[i, 0] = r
        cmap[i, 1] = g
        cmap[i, 2] = b
    # cmap = cmap.astype(np.float32) / 255
    return cmap
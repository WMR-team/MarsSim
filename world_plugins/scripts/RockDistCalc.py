# -*- coding: UTF-8 -*-
import numpy as np
import matplotlib.pyplot as plt
import random
from matplotlib.patches import Circle

def rock_distribution_function(k, D):
    '''岩石分布模型'''
    q = 1.79+0.152/k
    F = k*np.exp(-q*D)
    return F

def calculate_N(F, s):
    '''计算岩石数量'''
    return int(F*s)

def get_distance(x1, y1, x2, y2):
    '''计算两点之间的距离'''
    return np.sqrt((x1-x2)**2+(y1-y2)**2)

def check_collision(rock_list, x, y, D):
    '''检查新生成的岩石与原来生成的岩石是否存在冲突'''
    if len(rock_list) > 0:
        for i in range(len(rock_list)):
            x1 = rock_list[i]['x']
            y1 = rock_list[i]['y']
            D1 = rock_list[i]['D']
            dis = get_distance(x1, y1, x, y)
            if dis > D1+D:
                return False
            else:
                return True
    else:
        return False
    
# 计算岩石分布
def calculate_rock_distribution(DEM, param_data, terrain_class_mat, rock_dis_rate=None, return_record={}, is_show=False):
    '''计算岩石分布
    
    params:
        DEM: 地形DEM
        param_data: 预设参数字典
        terrain_class_mat: 地形类别矩阵
        is_show: 是否可视化展示岩石分布
    return:
        rock_list: 岩石分布列表，[{'x': , 'y': , 'D': },{}]
    '''
    # rock_size_list = [0.04, 0.08, 0.16, 0.32, 0.64, 1.28, 2.56, 5.12, 10.24]
    rock_size_list = [0.4, 0.8, 1.6, 3.2, 6.4]
    # rock_size_list = [0.3, 0.6, 0.9, 1.2, 1.5, 1.8, 2.1]
    rock_size_list = rock_size_list[-1::-1]
    terrain_classes = param_data['terrain_classes']
    # TODO: 不同地形岩石密度改变
    k_range = [0, 0.0]
    # k_list = [0.02+random.random()*k_range[1] for _ in range(terrain_classes)]
    k_list = [0.07 for _ in range(terrain_classes)]
    if rock_dis_rate != None:
        k_list = [rock_dis_rate for _ in range(terrain_classes)]
    return_record['rock_distri'] = k_list
    rock_list = []
    F_last = 0
    if is_show:
        fig = plt.figure()
        ax = fig.add_subplot(111)

    H = DEM.shape[1]
    min_l = DEM[0, 0, 0]
    max_l = DEM[0, 0, -1]
    l = max_l-min_l
    step = DEM[0, 0, 1] - DEM[0, 0, 0]

    area_list = []
    for i in range(terrain_classes):
        area_list.append(np.sum(terrain_class_mat == i)/(H*H)*l*l)
    for c in range(terrain_classes):
        for i in range(len(rock_size_list)):
            D = rock_size_list[i]
            # 计算该直径岩石的分布函数
            F = rock_distribution_function(k_list[c], D)
            F = F-F_last
            # 计算该直径岩石在该地形的个数
            N = calculate_N(F, area_list[c])
            # if D==0.6:
            #     N = N*2
            # elif D==0.2:
            #     N = N/2 
            
            j = 0
            while(j < N):
                x = random.random()*(l-4)+2
                y = random.random()*(l-4)+2
                # x = random.random()*l
                # y = random.random()*l
                flag_2 = (np.sqrt((x-l/2)**2+(y-l/2)**2)<4) or (np.sqrt((x-l/2)**2+(y-l/2)**2)>68 and np.sqrt((x-l/2)**2+(y-l/2)**2)<72)
                flag = check_collision(rock_list, x, y, D)
                # 如果有冲突，继续循环
                if flag:
                    pass
                if flag_2:
                    j +=1
                # 如果没有冲突，就将该点添加进列表，同时j+1
                else:
                    if terrain_class_mat[int(x/step), int(y/step)] == c:
                        rock = {}
                        rock['x'] = x
                        rock['y'] = y
                        rock['D'] = D
                        rock_list.append(rock)
                        j = j+1
                        cir = Circle(xy=(x, y), radius=D*2, alpha=0.5)
                        if is_show:
                            ax.add_patch(cir)
                    else:
                        pass
            F_last = F+F_last
    if is_show:
        plt.axis('scaled')
        # changes limits of x or y axis so that equal increments of x and y have the same length
        plt.axis('equal')
        ax.set_xlim(0, l)
        ax.set_ylim(0, l)
        plt.show()

    return rock_list
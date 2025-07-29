import numpy as np
import open3d as o3d
from matplotlib import cm
import matplotlib.pyplot as plt
import cv2
import csv
import os
import shutil
from colorama import Fore, Back, Style


def getArcCenterC(B, alpha_c, x_D, r_c):
    A = B * alpha_c * x_D**(alpha_c - 1)
    sqrt_term = np.sqrt(1 + A**2)
    x0 = x_D + (A * r_c) / sqrt_term
    y0 = B * x_D**alpha_c - r_c / sqrt_term
    return x0, y0

def computeH(xF, a2, R2):
    H = ((xF - a2)**2) / (R2**2 - (xF - a2)**2)
    return H

def getArcCenterF(H, r_c, r_f, xF, a2, b2):
    sqrt_term = np.sqrt(1 + H)
    x0 = np.sqrt(H) * r_f / sqrt_term + xF
    tmp_term = np.sqrt(r_c**2 - (xF - a2)**2)
    y0 = b2 + tmp_term + r_f / sqrt_term
    return x0, y0


def getCraterCurve(D,
                   alpha_c,
                   ratio,
                   d_multi=0.205,
                   d_power=1.012,
                   xi_c_multi=0.329,
                   xi_c_power=0.65,
                   xi_f_multi=0.273,
                   xi_f_power=0.85):
    '''
    陨石坑2D轮廓生成
    @params:
        D:陨石坑直径
        d_multi: 陨石坑深度乘子                     0.205, 0.045
        d_power: 陨石坑深度幂次                     1.012, 1.05
        alpha_c:陨石坑腔弧度指数                    [1.0, 2.0]        1.6
        xi_c_multi:陨石坑过渡段内侧 DF 乘子          0.329 ± 0.045     0.329
        xi_c_power:陨石坑过渡段内侧 DF 幂次          0.851 ± 0.027     0.551
        xi_f_multi:陨石坑过渡段外侧 FG 乘子          0.273 ± 0.0028    0.273
        xi_f_power:陨石坑过渡段外侧 FG 幂次          1.007 ± 0.013     0.650
        
    '''

    R = D / 2
    x_D = ratio * (D/2)
    d = d_multi * (D ** d_power)
    
    B = d / ((D / 2) ** alpha_c)

    xi_c = xi_c_multi * D ** xi_c_power
    xi_f = xi_f_multi * D ** xi_f_power
    r_c = 1 / xi_c
    r_f = 1 / xi_f

    delta_DF = (1 - ratio) * R
    xF = x_D + delta_DF

    # 计算 DF 圆心
    x0_DF, y0_DF = getArcCenterC(B, alpha_c, x_D, r_c)

    # 计算 FG 圆心
    H = computeH(xF, x0_DF, r_c)
    x0_FG, y0_FG = getArcCenterF(H, r_c, r_f, xF, x0_DF, y0_DF)

    # 构造 2D 曲线
    x_AD = np.linspace(0, x_D, 100)
    x_DF = np.linspace(x_D, xF, 50)
    x_FG = np.linspace(xF, x0_FG, 100)

    y_AD = B * x_AD**alpha_c
    y_DF = y0_DF + np.sqrt(r_c**2 - (x_DF - x0_DF)**2)
    y_FG = y0_FG - np.sqrt(r_f**2 - (x_FG - x0_FG)**2)

    x_all = np.concatenate((x_AD, x_DF, x_FG))
    y_all = np.concatenate((y_AD, y_DF, y_FG))
    end = y_all[-1]
    
    return B, alpha_c, x_D, x0_DF, y0_DF, r_c, xF, x0_FG, y0_FG, r_f, x0_FG, end

def normalizeUint16(array):
    """
    将数组归一化到 0-255 的范围，并转换为整数。
    :param array: 输入数组
    :return: 归一化后的数组
    """
    # 确保输入是 NumPy 数组
    array = np.asarray(array, dtype=np.float32)
    
    # 找到最小值和最大值
    min_val = np.min(array)
    max_val = np.max(array)
    
    # 避免除以零的情况
    if max_val == min_val:
        return np.zeros_like(array, dtype=np.uint16)
    
    # 线性映射到 [0, 255]
    normalized_array = (array - min_val) / (max_val - min_val) * (2**16-1) // 2
    
    # 转换为整数类型
    normalized_array = np.round(normalized_array)
    print("first num:", normalized_array[0])
    
    return normalized_array


def getCraterHM(D, B, alpha_c, x_D, x0_DF, y0_DF, r_c, x_F, x0_FG, y0_FG, r_f, x_G, end, num_x=100):
    '''
    AD: y = B * x ** alpha_c
    DF: y = y0 + sqrt(r_c ** 2 - (x - x0) ** 2)
    FG: y = y1 - sqrt(r_f ** 2 - (x - x1) ** 2)
    
    '''

    z_list = []
    for i in np.linspace(-100, 100, 2*num_x):
        for j in np.linspace(-100, 100, 2*num_x):
            x = i * x_G / (num_x - 1)
            y = j * x_G / (num_x - 1)
            
            r = np.sqrt(x ** 2 + y ** 2)
            if r <= x_D:
                z = B * r ** alpha_c 
            elif r <= x_F:
                dx = r - x0_DF
                z = y0_DF + np.sqrt(r_c ** 2 - dx ** 2) 
                
            elif r <= x_G:
                dx = r - x0_FG
                z = y0_FG - np.sqrt(r_f ** 2 - dx ** 2) 
                
            else:
                z = end
            
            z_list.append(z)
            
    z_list = np.array(z_list)
    z_list = normalizeUint16(z_list)
    print("z_list max min", z_list.max(), z_list.min())
    print(z_list.shape)
    z_list = z_list.reshape(2*num_x, 2*num_x) 
    
    width = int(100 * D / 10)
    
    z_list = cv2.resize(z_list, (width, width), interpolation=cv2.INTER_LINEAR) / 5  
    z_list = z_list - z_list[0]

    return z_list


def getCraterRotCurve(D,
                      alpha_c,
                      ratio,
                      d_multi=0.205,
                      d_power=1.012,
                      xi_c_multi=0.329,
                      xi_c_power=0.65,
                      xi_f_multi=0.273,
                      xi_f_power=0.85):
    '''
    陨石坑2D轮廓生成
    @params:
        D:陨石坑直径
        d_multi: 陨石坑深度乘子                     0.205, 0.045
        d_power: 陨石坑深度幂次                     1.012, 1.05
        alpha_c:陨石坑腔弧度指数                    [1.0, 2.0]        1.6
        xi_c_multi:陨石坑过渡段内侧 DF 乘子          0.329 ± 0.045     0.329
        xi_c_power:陨石坑过渡段内侧 DF 幂次          0.851 ± 0.027     0.551
        xi_f_multi:陨石坑过渡段外侧 FG 乘子          0.273 ± 0.0028    0.273
        xi_f_power:陨石坑过渡段外侧 FG 幂次          1.007 ± 0.013     0.650
        
    '''

    ratio = 0.9
    R = D / 2
    x_D = ratio * (D/2)
    d = d_multi * (D ** d_power)
    
    B = d / ((D / 2) ** alpha_c)

    xi_c = xi_c_multi * D ** xi_c_power
    xi_f = xi_f_multi * D ** xi_f_power
    r_c = 1 / xi_c
    r_f = 1 / xi_f

    delta_DF = (1 - ratio) * R
    xF = x_D + delta_DF

    # 计算 DF 圆心
    x0_DF, y0_DF = getArcCenterC(B, alpha_c, x_D, r_c)

    # 计算 FG 圆心
    H = computeH(xF, x0_DF, r_c)
    x0_FG, y0_FG = getArcCenterF(H, r_c, r_f, xF, x0_DF, y0_DF)

    # 构造 2D 曲线
    x_AD = np.linspace(0, x_D, 100)
    x_DF = np.linspace(x_D, xF, 50)
    x_FG = np.linspace(xF, x0_FG, 100)

    y_AD = B * x_AD**alpha_c
    y_DF = y0_DF + np.sqrt(r_c**2 - (x_DF - x0_DF)**2)
    y_FG = y0_FG - np.sqrt(r_f**2 - (x_FG - x0_FG)**2)

    x_all = np.concatenate((x_AD, x_DF, x_FG))
    y_all = np.concatenate((y_AD, y_DF, y_FG))
    
    return x_all, y_all



def getCraterSurf(x_all, y_all, num_theta=100):
    '''
    陨石坑3D 网格生成：2D曲线绕 y 轴旋转
    '''
    num_theta = 100  # 绕轴采样点数
    theta = np.linspace(0, 2*np.pi, num_theta)
    
    points = []
    triangles = []
    colors = []
    # 将 2D 曲线拼接为 (r, z) 坐标列表，表示从中心向外的剖面（z 表示高度）
    curve_rz = np.column_stack((x_all, y_all))  # r: 径向距离，z: 高度

    for i in range(len(curve_rz)):
        r, z = curve_rz[i]
        for th in theta:
            px = r * np.cos(th)
            py = z
            pz = r * np.sin(th)
            points.append([px, py, pz])
            


    min_z = np.min(y_all)
    max_z = np.max(y_all)
    cmap = cm.get_cmap("coolwarm")  

    for i in range(len(curve_rz)):
        r, z = curve_rz[i]
        color = cmap((z - min_z) / (max_z - min_z))[:3]  # 取 RGB，忽略透明度
        for _ in range(num_theta):
            colors.append(color)

    for i in range(len(curve_rz) - 1):
        for j in range(num_theta):
            p0 = i * num_theta + j
            p1 = i * num_theta + (j + 1) % num_theta
            p2 = (i + 1) * num_theta + (j + 1) % num_theta
            p3 = (i + 1) * num_theta + j

            triangles.append([p0, p1, p2])
            triangles.append([p0, p2, p3])
            
    return points, triangles, colors


def vis3D(points, triangles, colors):
    mesh = o3d.geometry.TriangleMesh()
    mesh.vertices = o3d.utility.Vector3dVector(points)
    mesh.triangles = o3d.utility.Vector3iVector(triangles)
    mesh.vertex_colors = o3d.utility.Vector3dVector(colors)

    o3d.visualization.draw_geometries([mesh])


def plot2D(x_all, y_all):
    plt.figure(figsize=(8, 4))
    plt.plot(x_all, y_all, label='Crater Profile')
    plt.xlabel("Radial Distance")
    plt.ylabel("Height")
    plt.title("Crater 2D Profile")
    plt.grid(True)
    plt.axis('equal')
    plt.legend()
    plt.show()
    


def addCrater2Terrain(terrain, crater_heightmap, x_offset=0, y_offset=0):
    '''
    将陨石坑高度图叠加到地形图上
    :param terrain: 地形图像（二维 numpy 数组）
    :param crater_heightmap: 陨石坑高度图
    :param x_offset: 相对于地形中心的横向偏移
    :param y_offset: 相对于地形中心的纵向偏移
    :return: 新的地形图
    '''
    h, w = crater_heightmap.shape
    th, tw = terrain.shape
    
    start_x = tw // 2 + x_offset - w // 2
    start_y = th // 2 + y_offset - h // 2
    
    for i in range(h):
        for j in range(w):
            x = start_x + i
            y = start_y + j
            if 0 <= x < tw and 0 <= y < th:
                terrain[x, y] += crater_heightmap[i, j]
                
                
    return terrain

    
 
def overlayCrater(background_path, height_map, output_path, x_offset=0, y_offset=0):
    """
    将小图像叠加到大图像的右下角。
    
    :param background_path: 背景图像路径
    :param height_map: 小图像数组
    :param output_path: 输出图像路径
    """

    # background = cv2.imread(background_path, cv2.IMREAD_GRAYSCALE)  
    background = cv2.imread(background_path, cv2.IMREAD_ANYDEPTH | cv2.IMREAD_GRAYSCALE)

    print("bg max:", np.max(background))
    if background is None:
        raise FileNotFoundError(f"无法读取背景图像: {background_path}")

    assert background.ndim == 2, "背景图像必须是灰度图"

    
    # 获取图像尺寸
    bg_height, bg_width = background.shape
    overlay_height, overlay_width = height_map.shape
    
    # FLAG: 确保背景图像叠加的位置正确
    
    # 叠加在右下角
    # x_offset = bg_width - overlay_width
    # y_offset = bg_height - overlay_height
    
    empty_img = (background * 0).astype(np.uint16)
    empty_img[y_offset:y_offset + overlay_height, x_offset:x_offset + overlay_width] = height_map
    print(height_map[-1][-1])
    height_map = background.astype(np.uint16) + empty_img
    print("----------------")
    print(np.max(height_map)) 
    
    cv2.imwrite(output_path, height_map)
    print(f"叠加后的图像已保存到: {output_path}")

def isFeasible(x_offset, y_offset, width):
    
    bg_shape = (640, 640)
    if x_offset < 0 or y_offset < 0 or x_offset + width > bg_shape[1] or y_offset + width > bg_shape[0]:
        return False
    
    return True




def saveCraterInfo(crater_file, crater_x, crater_y, D, alpha_c, ratio):
    '''
    保存陨石坑信息
    '''

    with open(crater_file, "a", newline='', encoding='utf-8') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow([crater_x, crater_y, D, alpha_c, ratio])

    
def craterGenerator(crater_file, bg_path, output_640, output_513,
                    D_bound, alpha_c_bound, ratio_bound, x_offset_bound, y_offset_bound):
    '''
    陨石坑生成器
    '''    
    with open(crater_file, "w", newline='', encoding='utf-8') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(["crater_x", "crater_y", "D", "alpha_c", "ratio"])
    
    shutil.copy(bg_path, output_640)
    
    while True:
        D = np.random.uniform(D_bound[0], D_bound[1])
        alpha_c = np.random.uniform(alpha_c_bound[0], alpha_c_bound[1])
        ratio = np.random.uniform(ratio_bound[0], ratio_bound[1])
        x_offset = int(np.random.uniform(x_offset_bound[0], x_offset_bound[1]))
        y_offset = int(np.random.uniform(y_offset_bound[0], y_offset_bound[1]))
        print(Fore.LIGHTGREEN_EX+ f"坑缘直径: {D}, 坑腔幂律指数: {alpha_c}, 边缘系数: {ratio}, X坐标: {x_offset}, Y坐标: {y_offset}" + Style.RESET_ALL)
        
        
        B, alpha_c, x_D, x0_DF, y0_DF, r_c, xF, x0_FG, y0_FG, r_f, x0_FG, end = getCraterCurve(D, 
                                                                                            alpha_c, 
                                                                                            ratio)
        x_all, y_all = getCraterRotCurve(D,
                                    alpha_c,
                                    ratio)
        
        points, triangles, colors = getCraterSurf(x_all=x_all, y_all=y_all)
        height_map = getCraterHM(D, B, alpha_c, x_D, x0_DF, y0_DF, r_c, xF, x0_FG, y0_FG, r_f, x0_FG, end)

        vis3D(points, triangles, colors)
        visCraterHM(height_map)
        
        width = height_map.shape[0]
        
        decision = input("Press s to SAVE; Press c to CONTINUE WITHOUT SAVING; Press others to EXIT.")
        if decision == "s":
            # FLAG: 记录陨石坑的位置和参数作为ground truth
            print("Saving...")
            if isFeasible(x_offset, y_offset, width):
                saveCraterInfo(crater_file, x_offset, y_offset, D, alpha_c, ratio)
                overlayCrater(output_640, height_map, output_640, x_offset, y_offset)
            else:
                print("Infeasible crater parameters.")
                print("Continue...")
                continue
        elif decision == "c":
            print("Continue...")
            continue
        else:
            print("Exiting...")
            break
        
        print("------------------------------------------")
        
    saveResizeHM(output_640, output_513)
    
    

def saveResizeHM(input_path, output_path):
    
    # 读取 uint16 灰度图
    img = cv2.imread(input_path, cv2.IMREAD_ANYDEPTH | cv2.IMREAD_GRAYSCALE)

    if img is None:
        print("图像读取失败，请检查路径或文件格式。")
    else:
        print("原始图像数据类型:", img.dtype)  # 应为 uint16
        print("原始图像尺寸:", img.shape)      # 应为 (640, 640)

        # 归一化到 0~255 并转换为 uint8
        img_uint8 = cv2.normalize(img, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)

        # 调整尺寸为 513x513，使用线性插值
        resized_img = cv2.resize(img_uint8, (513, 513), interpolation=cv2.INTER_LINEAR)

        print(np.max(resized_img), np.min(resized_img))

        cv2.imwrite(output_path, resized_img)
        print(f"513 图像已保存至: {output_path}")

def visCraterHM(height_map):
    plt.figure(figsize=(6, 6))
    plt.imshow(height_map.T, cmap='gray', origin='lower')
    plt.colorbar(label='Height (negative is crater)')
    print(height_map.shape)
    plt.title("Crater Heightmap")
    plt.show()
    
def getOutputFile(bg_path):
    current_dir = os.path.dirname(os.path.abspath(__file__))
    dir_16 = os.path.dirname(os.path.abspath(bg_path))
    dir_8 = os.path.join(os.path.dirname(dir_16), "heightmaps_int8")
    base_name = os.path.basename(bg_path)
    name, ext = os.path.splitext(base_name)
    crater_file_16 = os.path.join(dir_16, f"{name}_crater{ext}")
    crater_file_8 = os.path.join(dir_8, f"{name}_crater{ext}")

    crater_file = os.path.join(current_dir, "crater_info.csv")
    
    return crater_file_16, crater_file_8, crater_file
    
'''
陨石坑2D轮廓生成
@params:
    D:陨石坑直径
    d_multi: 陨石坑深度乘子                     0.205, 0.045
    d_power: 陨石坑深度幂次                     1.012, 1.05
    alpha_c:陨石坑腔弧度指数                    [1.0, 2.0]        1.6
    xi_c_multi:陨石坑过渡段内侧 DF 乘子          0.329 ± 0.045     0.329
    xi_c_power:陨石坑过渡段内侧 DF 幂次          0.851 ± 0.027     0.551
    xi_f_multi:陨石坑过渡段外侧 FG 乘子          0.273 ± 0.0028    0.273
    xi_f_power:陨石坑过渡段外侧 FG 幂次          1.007 ± 0.013     0.650
    
'''


D_bound = [2, 10]
alpha_c_bound = [1.3, 2.0]
ratio_bound = [0.75, 1.0]
x_offset_bound = [10, 513]
y_offset_bound = [10, 513]

'''
设置背景图路径，例如默认如下
bg_path = "/home/tang123/MarsSim_v2_ws/src/rover_gazebo/models/mars_terrain/heightmaps_int16/HM5.png"

'''
bg_path = ""
output_640, output_513, crater_file = getOutputFile(bg_path)


craterGenerator(crater_file, bg_path, output_640, output_513,
                D_bound, alpha_c_bound, ratio_bound, x_offset_bound, y_offset_bound)



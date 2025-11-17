import cv2
import numpy as np
from sklearn.cluster import KMeans
from sklearn.metrics import silhouette_score
import matplotlib.pyplot as plt
import os
import glob

def batch_adaptive_clustering(input_folder, output_folder, max_classes=10, min_region_ratio=0.001):
    """
    批量处理文件夹中的图片，进行改进的自适应聚类
    """
    # 创建输出文件夹
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)
    
    # 支持的图片格式
    image_extensions = ['*.jpg', '*.jpeg', '*.png', '*.bmp', '*.tiff', '*.tif']
    image_paths = []
    
    # 获取所有图片文件路径
    for extension in image_extensions:
        image_paths.extend(glob.glob(os.path.join(input_folder, extension)))
        image_paths.extend(glob.glob(os.path.join(input_folder, extension.upper())))
    
    print(f"找到 {len(image_paths)} 张图片需要处理")
    
    # 处理每张图片
    for i, image_path in enumerate(image_paths):
        try:
            print(f"正在处理 {i+1}/{len(image_paths)}: {os.path.basename(image_path)}")
            
            # 生成输出路径
            filename = os.path.basename(image_path)
            name_without_ext = os.path.splitext(filename)[0]
            output_path = os.path.join(output_folder, f"{name_without_ext}_clustered.png")
            
            # 处理单张图片
            result = improved_adaptive_clustering(image_path, output_path, max_classes, min_region_ratio)
            
            if result is not None:
                print(f"成功处理: {filename} -> {os.path.basename(output_path)}")
            else:
                print(f"处理失败: {filename}")
                
        except Exception as e:
            print(f"处理图片 {image_path} 时出错: {str(e)}")
    
    print("批量处理完成!")

def improved_adaptive_clustering(image_path, output_path, max_classes=10, min_region_ratio=0.001):
    """
    改进的自适应聚类方法，智能处理小区域和边缘像素
    """
    # 读取图像
    img = cv2.imread(image_path)
    if img is None:
        print(f"错误: 无法读取图像 {image_path}")
        return None
    
    h, w, c = img.shape
    img_flat = img.reshape((-1, 3))
    
    # 自动确定最佳类别数量
    best_k = find_optimal_clusters_silhouette(img_flat, max_classes)
    print(f"检测到 {best_k} 个类别")
    
    # 使用K-means聚类
    kmeans = KMeans(n_clusters=best_k, random_state=42, n_init=10)
    labels = kmeans.fit_predict(img_flat)
    clustered = labels.reshape((h, w))
    
    # 分析每个聚类区域
    region_info = analyze_regions(clustered, best_k, min_region_ratio, h * w)
    
    # 智能区域合并和保留
    refined_labels = smart_region_refinement(clustered, region_info, kmeans.cluster_centers_)
    
    # 创建结果图像
    result = np.zeros_like(img)
    for i in range(best_k):
        mask = (refined_labels == i)
        if np.sum(mask) > 0:  # 只处理存在的类别
            result[mask] = kmeans.cluster_centers_[i].astype(int)
    
    # 保存为PNG格式
    cv2.imwrite(output_path, result)
    
    # 显示结果对比（可选，批量处理时可以注释掉以加快速度）
    # show_comparison(img, result, "改进的自适应聚类")
    
    return result

def find_optimal_clusters_silhouette(data, max_k):
    """
    使用轮廓系数确定最佳聚类数量
    """
    if len(data) > 10000:  # 对大数据集进行采样
        indices = np.random.choice(len(data), 10000, replace=False)
        sample_data = data[indices]
    else:
        sample_data = data
    
    best_score = -1
    best_k = 2
    
    for k in range(2, min(max_k + 1, 8)):  # 限制最大尝试数量
        if len(sample_data) < k:
            continue
            
        kmeans = KMeans(n_clusters=k, random_state=42, n_init=5)
        labels = kmeans.fit_predict(sample_data)
        
        if len(np.unique(labels)) < 2:
            continue
            
        try:
            score = silhouette_score(sample_data, labels)
            print(f"K={k}, 轮廓系数: {score:.4f}")
            
            if score > best_score:
                best_score = score
                best_k = k
        except:
            continue
    
    # 如果轮廓系数都不好，使用肘部法则
    if best_score < 0.3:
        best_k = find_optimal_clusters_elbow(data, max_k)
    
    return max(2, best_k)  # 至少2个类别

def find_optimal_clusters_elbow(data, max_k):
    """
    使用肘部法则确定最佳聚类数量
    """
    inertias = []
    k_range = range(2, min(max_k + 1, 10))
    
    for k in k_range:
        kmeans = KMeans(n_clusters=k, random_state=42, n_init=5)
        kmeans.fit(data)
        inertias.append(kmeans.inertia_)
    
    # 计算二阶差分寻找拐点
    if len(inertias) >= 3:
        first_diff = np.diff(inertias)
        second_diff = np.diff(first_diff)
        
        if len(second_diff) > 0:
            elbow_point = np.argmax(second_diff) + 2  # +2 因为从k=2开始
            return min(elbow_point, max_k)
    
    return 3  # 默认值

def analyze_regions(clustered_labels, k, min_ratio, total_pixels):
    """
    分析每个聚类区域的特性
    """
    region_info = {}
    min_region_size = int(total_pixels * min_ratio)
    
    for i in range(k):
        mask = (clustered_labels == i)
        region_size = np.sum(mask)
        
        # 计算区域的紧凑度（轮廓面积/边界框面积）
        region_mask = mask.astype(np.uint8) * 255
        contours, _ = cv2.findContours(region_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        compactness = 0
        if len(contours) > 0:
            largest_contour = max(contours, key=cv2.contourArea)
            contour_area = cv2.contourArea(largest_contour)
            x, y, w, h = cv2.boundingRect(largest_contour)
            bbox_area = w * h
            compactness = contour_area / bbox_area if bbox_area > 0 else 0
        
        # 计算边界强度（边缘像素比例）
        edges = cv2.Canny(region_mask, 50, 150)
        edge_ratio = np.sum(edges > 0) / region_size if region_size > 0 else 0
        
        region_info[i] = {
            'size': region_size,
            'compactness': compactness,
            'edge_ratio': edge_ratio,
            'is_small': region_size < min_region_size,
            'is_noise': region_size < max(50, min_region_size // 2),  # 更小的区域被认为是噪声
            'is_edge_region': edge_ratio > 0.3  # 边界区域
        }
    
    return region_info

def smart_region_refinement(clustered_labels, region_info, cluster_centers):
    """
    智能区域合并和保留策略
    """
    refined = clustered_labels.copy()
    k = len(cluster_centers)
    
    # 第一步：合并非常小的噪声区域到最近的邻居
    for i in range(k):
        if region_info[i]['is_noise']:
            # 找到最近的邻居
            nearest_label = find_nearest_cluster(i, cluster_centers, excluded=[i])
            if nearest_label is not None:
                refined[refined == i] = nearest_label
                print(f"合并噪声区域 {i} 到 {nearest_label}")
    
    # 第二步：处理小但紧凑的区域（可能是重要的小对象）
    for i in range(k):
        if (region_info[i]['is_small'] and 
            region_info[i]['compactness'] > 0.7 and  # 紧凑的形状
            not region_info[i]['is_edge_region']):   # 不是边界区域
            # 保留这个小区域
            print(f"保留紧凑小区域 {i} (大小: {region_info[i]['size']}, 紧凑度: {region_info[i]['compactness']:.2f})")
    
    # 第三步：合并边界模糊区域
    for i in range(k):
        if (region_info[i]['is_edge_region'] and 
            region_info[i]['compactness'] < 0.3):  # 不紧凑的边界区域
            nearest_label = find_nearest_cluster(i, cluster_centers, excluded=[i])
            if nearest_label is not None:
                refined[refined == i] = nearest_label
                print(f"合并边界模糊区域 {i} 到 {nearest_label}")
    
    return refined

def find_nearest_cluster(target_label, cluster_centers, excluded=[]):
    """
    找到颜色最接近的聚类（排除指定标签）
    """
    min_distance = float('inf')
    nearest_label = None
    target_center = cluster_centers[target_label]
    
    for i, center in enumerate(cluster_centers):
        if i == target_label or i in excluded:
            continue
            
        # 计算颜色距离（欧氏距离）
        distance = np.linalg.norm(target_center - center)
        if distance < min_distance:
            min_distance = distance
            nearest_label = i
    
    return nearest_label

def show_comparison(original, result, title):
    """
    显示原始图像和处理后的图像对比
    """
    plt.figure(figsize=(12, 6))
    
    plt.subplot(1, 2, 1)
    plt.imshow(cv2.cvtColor(original, cv2.COLOR_BGR2RGB))
    plt.title('原图')
    plt.axis('off')
    
    plt.subplot(1, 2, 2)
    plt.imshow(cv2.cvtColor(result, cv2.COLOR_BGR2RGB))
    plt.title(title)
    plt.axis('off')
    
    plt.tight_layout()
    plt.show()

# 使用示例
if __name__ == "__main__":
    # 设置输入和输出文件夹路径
    input_folder = "input_folder"  # 输入文件夹，包含要处理的图片
    output_folder = "output_folder"  # 输出文件夹，处理后的图片将保存到这里
    
    # 批量处理文件夹中的所有图片
    batch_adaptive_clustering(
        input_folder=input_folder,
        output_folder=output_folder,
        max_classes=5,
        min_region_ratio=0.001
    )

#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import os
import time
import subprocess
import sys
import numpy as np
from std_msgs.msg import Bool, Int32
from sensor_msgs.msg import Image

class ImageSaver:
    def __init__(self, save_base_path, epoch):
        self.save_base_path = save_base_path
        self.epoch = epoch
        
        # 状态变量
        self.current_label_mode = False
        self.current_pose_index = -1
        self.is_processing = False
        
        # 图像缓存
        self.latest_image = None
        self.latest_image_time = None
        
        # 订阅控制话题
        self.control_sub = rospy.Subscriber('/simulation_control', Int32, self.control_callback)
        self.label_sub = rospy.Subscriber('/is_label_mode', Bool, self.label_callback)
        
        # 检测并订阅图像话题
        self.image_topic = self.detect_image_topic()
        if self.image_topic:
            print(f"✅ 检测到图像话题: {self.image_topic}")
            # 持续订阅图像话题，用于缓存最新图像
            self.image_sub = rospy.Subscriber(self.image_topic, Image, self.image_callback)
        else:
            print("❌ 未找到图像话题")
            return
        
        # 完成确认发布器
        self.completion_pub = rospy.Publisher('/save_completion', Int32, queue_size=1)
        
        # 检查PIL依赖
        self.check_pil_dependency()
        
        print(f"✅ 优化版图像保存器已启动")
        print(f"   保存路径: {self.save_base_path}")
        print(f"   Epoch: {self.epoch}")
        print(f"   图像话题: {self.image_topic}")
        print("⏳ 等待控制信号...")
    
    def check_pil_dependency(self):
        """检查PIL依赖"""
        try:
            from PIL import Image as PILImage
            self.pil_available = True
            print("✅ PIL/Pillow 已安装")
        except ImportError:
            self.pil_available = False
            print("❌ PIL/Pillow 未安装，无法保存图像")
            print("   请运行: pip install Pillow")
    
    def detect_image_topic(self, timeout=10.0):
        """检测图像话题"""
        start_time = time.time()
        while time.time() - start_time < timeout and not rospy.is_shutdown():
            try:
                topics = rospy.get_published_topics()
                image_topics = [topic[0] for topic in topics if topic[1] == 'sensor_msgs/Image']
                
                # 优先选择包含camera/image_raw的话题
                for topic in image_topics:
                    if 'camera' in topic and 'image_raw' in topic:
                        return topic
                
                # 如果没有，选择第一个图像话题
                if image_topics:
                    return image_topics[0]
                    
            except Exception as e:
                print(f"检测话题时出错: {e}")
            time.sleep(0.5)
        return None
    
    def image_callback(self, msg):
        """图像回调 - 持续缓存最新图像"""
        self.latest_image = msg
        self.latest_image_time = rospy.Time.now()
    
    def label_callback(self, msg):
        """标签模式回调"""
        self.current_label_mode = msg.data
        print(f"📝 设置标签模式: {self.current_label_mode}")
    
    def control_callback(self, msg):
        """控制信号回调 - 立即处理并发送确认"""
        if self.is_processing:
            print(f"⚠️  正在处理前一个请求，忽略索引: {msg.data}")
            return
            
        self.current_pose_index = msg.data
        print(f"📨 收到保存命令，开始处理索引: {self.current_pose_index}")
        
        # 处理保存
        success = self.process_current_pose()
        
        # 发送完成确认
        completion_msg = Int32()
        completion_msg.data = self.current_pose_index
        self.completion_pub.publish(completion_msg)
        print(f"📤 发送完成确认: {self.current_pose_index} ({'成功' if success else '失败'})")
    
    def process_current_pose(self):
        """处理当前位姿的图像保存"""
        self.is_processing = True
        
        try:
            # 确定保存目录
            if self.current_label_mode:
                save_dir = os.path.join(self.save_base_path, f'epoch_{self.epoch:03d}', 'labels')
            else:
                save_dir = os.path.join(self.save_base_path, f'epoch_{self.epoch:03d}', 'originals')
            
            os.makedirs(save_dir, exist_ok=True)
            filename = f"pose_{self.current_pose_index:04d}.jpg"
            filepath = os.path.join(save_dir, filename)
            
            print(f"🖼️  开始保存图像: {filepath}")
            
            # 使用优化的保存方法
            success = self.save_image_fast(filepath)
            
            if success:
                print(f"✅ 图像保存成功: {filepath}")
                # 验证文件
                if self.verify_saved_file(filepath):
                    print(f"✅ 文件验证通过")
                else:
                    print(f"⚠️  文件验证失败")
            else:
                print(f"❌ 图像保存失败: {filepath}")
                
            return success
            
        except Exception as e:
            print(f"❌ 保存过程异常: {e}")
            import traceback
            traceback.print_exc()
            return False
        finally:
            self.is_processing = False
    
    def save_image_fast(self, filepath, timeout=3):
        """快速保存图像 - 主要工作方法"""
        if not self.pil_available:
            print("❌ PIL不可用，无法保存图像")
            return False
            
        try:
            # 首先尝试使用缓存图像（最快）
            if self.latest_image and self.is_image_fresh():
                print("  🚀 使用缓存图像快速保存")
                return self.process_image_data(self.latest_image, filepath)
            
            # 如果没有缓存或缓存太旧，等待一帧新图像
            print("  ⏳ 等待图像消息...")
            start_time = time.time()
            image_msg = rospy.wait_for_message(self.image_topic, Image, timeout=timeout)
            wait_time = time.time() - start_time
            print(f"  ✅ 接收到图像消息 (等待: {wait_time:.2f}s)")
            
            return self.process_image_data(image_msg, filepath)
            
        except rospy.ROSException as e:
            print(f"  ❌ 等待图像消息超时: {e}")
            return False
        except Exception as e:
            print(f"  ❌ 保存过程出错: {e}")
            return False
    
    def is_image_fresh(self, max_age=1.0):
        """检查缓存图像是否新鲜"""
        if not self.latest_image_time:
            return False
        
        age = (rospy.Time.now() - self.latest_image_time).to_sec()
        return age < max_age
    
    def process_image_data(self, image_msg, filepath):
        """处理图像数据并保存 - 使用PIL"""
        try:
            from PIL import Image as PILImage
            
            print(f"    编码: {image_msg.encoding}, 尺寸: {image_msg.width}x{image_msg.height}")
            print(f"    数据大小: {len(image_msg.data)} 字节")
            
            # 将数据转换为numpy数组
            if image_msg.encoding == 'rgb8':
                # RGB格式
                image_data = np.frombuffer(image_msg.data, dtype=np.uint8)
                image_data = image_data.reshape((image_msg.height, image_msg.width, 3))
                pil_image = PILImage.fromarray(image_data, 'RGB')
                
            elif image_msg.encoding == 'bgr8':
                # BGR格式，需要转换为RGB
                image_data = np.frombuffer(image_msg.data, dtype=np.uint8)
                image_data = image_data.reshape((image_msg.height, image_msg.width, 3))
                # 将BGR转换为RGB
                image_data = image_data[:, :, [2, 1, 0]]  # BGR -> RGB
                pil_image = PILImage.fromarray(image_data, 'RGB')
                
            elif image_msg.encoding == 'mono8':
                # 单通道灰度图像
                image_data = np.frombuffer(image_msg.data, dtype=np.uint8)
                image_data = image_data.reshape((image_msg.height, image_msg.width))
                pil_image = PILImage.fromarray(image_data, 'L')
            
            elif image_msg.encoding == '32FC1':
                # 浮点深度图像，需要转换为8位
                image_data = np.frombuffer(image_msg.data, dtype=np.float32)
                image_data = image_data.reshape((image_msg.height, image_msg.width))
                # 归一化到0-255
                image_data = (image_data - np.min(image_data)) / (np.max(image_data) - np.min(image_data)) * 255
                image_data = image_data.astype(np.uint8)
                pil_image = PILImage.fromarray(image_data, 'L')
                
            else:
                print(f"    ❌ 不支持的图像编码: {image_msg.encoding}")
                return False
            
            # 保存图像（使用优化参数）
            pil_image.save(filepath, 'JPEG', quality=95, optimize=True)
            print(f"    ✅ 使用PIL保存成功: {filepath}")
            return True
            
        except ImportError:
            print("    ❌ PIL未安装，无法保存图像")
            return False
        except Exception as e:
            print(f"    ❌ PIL处理失败: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def verify_saved_file(self, filepath):
        """验证保存的文件"""
        try:
            if not os.path.exists(filepath):
                print(f"    ❌ 文件不存在: {filepath}")
                return False
            
            file_size = os.path.getsize(filepath)
            print(f"    📁 文件大小: {file_size} 字节")
            
            if file_size < 100:
                print(f"    ❌ 文件过小，可能有问题")
                return False
            
            # 检查文件扩展名
            if filepath.lower().endswith(('.png', '.jpg', '.jpeg')):
                # 尝试读取文件头验证是否为图像文件
                with open(filepath, 'rb') as f:
                    header = f.read(10)
                    # 检查JPEG文件头
                    if header.startswith(b'\xff\xd8\xff'):
                        print("    ✅ 文件头验证: JPEG格式")
                        return True
                    # 检查PNG文件头
                    elif header.startswith(b'\x89PNG\r\n\x1a\n'):
                        print("    ✅ 文件头验证: PNG格式")
                        return True
                    else:
                        print(f"    ⚠️  未知文件格式，文件头: {header.hex()}")
                        return True  # 仍然返回True，可能是其他格式
            else:
                # 非图像文件
                print("    📄 非图像文件")
                return False
                
        except Exception as e:
            print(f"    ❌ 文件验证异常: {e}")
            return False

def main():
    rospy.init_node('image_saver', anonymous=True)
    
    if len(sys.argv) < 3:
        print("❌ 用法: python image_saver_optimized.py <保存路径> <epoch>")
        print("📝 示例: python image_saver_optimized.py /home/li/MarsSim_v2_ws/src/MarsSim_v2-main/simulation_images 1")
        return
    
    save_base_path = sys.argv[1]
    epoch = int(sys.argv[2])
    
    print(f"🚀 启动优化版图像保存器")
    print(f"   保存路径: {save_base_path}")
    print(f"   Epoch: {epoch}")
    
    image_saver = ImageSaver(save_base_path, epoch)
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("\n🛑 图像保存器已停止")

if __name__ == "__main__":
    main()

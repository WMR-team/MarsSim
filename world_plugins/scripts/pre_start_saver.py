#!/usr/bin/env python
# -*- coding: utf-8 -*-
# pre_start_saver.py - 预启动图像保存器

import subprocess
import time
import sys
import os

def pre_start_image_saver(save_base_path, epoch):
    """预启动图像保存器"""
    print("🚀 预启动图像保存器...")
    
    # 启动image_saver节点
    cmd = f"python optimized_image_saver.py {save_base_path} {epoch}"
    process = subprocess.Popen(cmd, shell=True)
    
    print(f"✅ 图像保存器已启动 (PID: {process.pid})")
    print("⏳ 等待图像保存器初始化...")
    
    # 等待一段时间让保存器初始化
    time.sleep(5)
    
    return process

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("用法: python pre_start_saver.py <保存路径> <epoch>")
        sys.exit(1)
    
    save_base_path = sys.argv[1]
    epoch = int(sys.argv[2])
    
    process = pre_start_image_saver(save_base_path, epoch)
    
    try:
        print("图像保存器运行中，按Ctrl+C停止...")
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n🛑 停止图像保存器...")
        process.terminate()
        process.wait()

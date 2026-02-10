<!-- Project overview and setup instructions for MarsSim_v2. -->
# MarsSim_v2

## 概述

## 安装依赖

- Ubuntu 20.04
- Python3
- [ROS Neotic](http://wiki.ros.org/neotic/Installation/Ubuntu)
- catkin_tools
- pyyaml
- cv2
- numpy
- matplotlib
- pandas

## 教程

## 安装
### 1. 项目初始化
    $ mkdir MarsSim_v2_ws
    $ cd MarsSim_v2_ws
    $ mkdir src
    $ cd src
    $ git clone https://github.com/fengwh123/MarsSim_v2
    $ cd ..
    $ catkin build -DPYTHON_EXECUTABLE=/usr/bin/python3
    $ source ./devel/setup.bash

### 2. Download & install model assets (auto)
    $ cd ./src/MarsSim
    $ pip install hydra-core omegaconf
    $ python -m world_plugins.scripts.download_models --gdrive-file-id <YOUR_FILE_ID>

### 3. 更改路径
将文件中出现的绝对路径进行全部替换 

### 4. 运行地形生成文件

    $ cd src/world_plugins/
    $ python ./scripts/world_change_pipeline.py

### 5. 启动火星车仿真(二选一)
高保真场景

    $ roslaunch rover_gazebo zhurong_main_real.launch

简单场景

    $ roslaunch rover_gazebo zhurong_main_simple.launch

## 6. 键盘控制火星车移动
在terminal窗口中输入如下键盘控制指令,并回车，控制火星车移动：
- w: 前进
- s: 后退
- a: 左转
- d: 右转
- p: 停止
- TODO： `添加停止、加速、减速以及更多模式`



## 工程文件架构

- rover_control: 火星车底层控制配置
  - config
  - launch
  - scripts
- rover_descriptions: 火星车仿真三维urdf文件
  - triwheel_rover_descrition
  - zhurong_rover_description
- rover_gazebo: 火星车gazebo仿真
  - data
  - launch
  - models
  - rviz
  - urdf
  - worlds
- rover_gazebo_plugins: 火星车仿真插件
  - config
  - include
  - src
- world_plugins: 生成仿真场景的插件
  - config
  - scripts
  - src


## 致谢

## 引用
```
@InProceedings{,
	author    = {},
	title     = {{}},
	booktitle = {{}},
	year      = {}
}
```
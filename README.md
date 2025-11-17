# MarsSim_v2

## 概述

## 安装依赖

- Ubuntu 18.04
- [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)
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
    $ catkin build
    $ source ./devel/setup.bash

### 2. 找我拷贝模型文件

将文件解压后，放置于`~/MarsSim_v2/src/rover_gazebo/`文件夹中

### 3. 更改路径
将文件中出现的绝对路径进行全部替换 

### 4. 运行仿真与标签生成文件以及保存

    $ cd src/world_plugins/
    $ python ./scripts/simulation_pipeline.py
	在新终端打开
	$ cd src/world_plugins/
    $ python ./scripts/image_saver.py
	按照simulation_pipeline.py运行指令进行图片的保存

### 5. 保存图片的后处理
    $ cd src/world_plugins/
    $ python ./scripts/ppimage.py
	将输入输出文件夹更改为自己需要




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
- rover_navigation: 火星车导航程序包
  - config
  - launch
  - scripts
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

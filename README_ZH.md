<!-- MarsSim_v2 中文说明与安装使用指南。 -->
# MarsSim_v2

## 概述

## 安装依赖

- Ubuntu 20.04
- [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)
- catkin_tools
- pyyaml
- cv2
- numpy
- matplotlib
- pandas
- hydra-core
- omegaconf

## 教程

## 安装
### 1. 项目初始化
```shell
mkdir -p MarsSim_ws/src && cd MarsSim_ws
git clone --branch remove_abs_path git@github.com:WMR-team/MarsSim.git ./src/MarsSim
catkin build
source ./devel/setup.bash
```
### 2. 下载并安装模型资源
```shell
cd ./src/MarsSim
pip install hydra-core omegaconf

# 方式1：用 Google Drive file id（推荐）
python -m world_plugins.scripts.download_models --gdrive-file-id 1WT5JkZ87SlinNSlQP95LfcPy7OwLVmif

# 方式2：用 Google Drive 分享链接，下载后请手动将model解压至`Marsim/rover_gazebo`路径下，期望解压后的路径形式为:
Marsim
├── ...
├── rover_control
│   ├── ...
├── rover_descriptions
│   ├── ...
├── rover_gazebo
│   ├── ...
│   ├── models                     <-- Your extracted Folder
│   ├── ...
├── rover_gazebo_plugins
│   ├── ...
└── world_plugins
    ├── ...
```
[Google Drive 分享链接](https://drive.google.com/file/d/1WT5JkZ87SlinNSlQP95LfcPy7OwLVmif/view?usp=drive_link)
[百度网盘 分享链接](https://pan.baidu.com/s/1wRg5N2Vxj_nuMMZaTU_9PA?pwd=1234)

### 3. 运行地形生成文件
```shell
cd ./src/MarsSim
pip install hydra-core omegaconf
python -m world_plugins.scripts.world_change_pipeline
```
### 4. 启动火星车仿真(二选一)
```shell
# 高保真场景
roslaunch rover_gazebo zhurong_main_real.launch
# 简单场景
roslaunch rover_gazebo zhurong_main_simple.launch
```
## 5. 键盘控制火星车移动
在terminal窗口中输入如下键盘控制指令，控制火星车移动：
- `w`: 前进
- `s`: 后退
- `a`: 左转
- `d`: 右转
- TODO： `添加停止、加速、减速以及更多模式`



## 工程文件架构
```txt
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
```

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

# TODO:
- 绝对路径
- world文件云下载，脚本
- 代码规范
- 配置文件()
- log(hydra)

- 三轮车的模型好像有问题
- 场景自主生成(simple的进来和实际的对不上，main是固定的， 想要直接就能生成+打开一套，不弄那么负责)
- 数据采集()
- 需要有一个可视化的界面，把joystick啥的都，
- 最基本的utilize的脚本，rosbag 的记录和显示的脚本，想
-
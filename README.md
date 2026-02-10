<!-- MarsSim_v2 中文说明与安装使用指南。 -->
# MarsSim_v2

## 概述
MarsSim_v2 是一个基于 ROS + Gazebo 的火星车仿真与地形/岩石场景生成工程，包含：
- 地形高度图/纹理/语义类图生成
- 岩石分布与模型生成
- Terramechanics 插件（读取 DTM 文件）驱动轮地交互
- RViz 可视化与相机/传感器话题
<div align="center" style="margin: 20px 0;">
  <img src="assets/intro.png"
    alt="intro img"
    title="Marsim Gazebo World Environment"
    width="800"
    style="max-width: 100%; height: auto; border-radius: 8px; box-shadow: 0 4px 12px rgba(0,0,0,0.15);"
    loading="lazy"/>
</div>



## 环境依赖
- Ubuntu 20.04
- ROS Noetic
- Gazebo（随 ROS Noetic）
- Python3
- Python 包：`pyyaml`, `opencv-python`, `numpy`, `pandas`, `hydra-core`, `omegaconf`

## Quickstart（推荐）
### 1) 克隆并编译
```bash
mkdir -p MarsSim_ws/src && cd MarsSim_ws
git clone git@github.com:WMR-team/MarsSim.git ./src/MarsSim
catkin build
source ./devel/setup.bash
```

### 2) 安装 Python 依赖
```bash
python3 -m pip install pyyaml opencv-python numpy pandas hydra-core omegaconf
```

### 3) 下载并安装模型资源（自动）
模型资源不直接入库（体积较大）。请使用脚本自动下载并解压到：
`<repo>/rover_gazebo/models/`

```bash
cd ./src/MarsSim
python3 -m world_plugins.scripts.download_models --gdrive-file-id 1WT5JkZ87SlinNSlQP95LfcPy7OwLVmif
```

如果无法使用脚本自动下载，请在如下的网盘链接下载 `models.zip` 压缩包并手动解压至 `<repo>/rover_gazebo/models/` 目录下:

模型解压后的期望结构（示例）：
```txt
MarsSim
├── rover_gazebo
│   ├── models
│   │   ├── mars_terrain
│   │   ├── mars_rocks_lbl
│   │   ├── ai_camera
│   │   └── ...
│   └── ...
└── world_plugins
    └── ...
```

### 4) 生成地形与场景
```bash
cd ./src/MarsSim
python3 -m world_plugins.scripts.world_change_pipeline
```

### 5) 启动仿真（任选其一）
```bash
# 高保真场景
roslaunch rover_gazebo zhurong_main_real.launch

# 简单场景
roslaunch rover_gazebo zhurong_main_simple.launch
```


## 开发与贡献
### 代码格式化
- Python：`black`
- C/C++：`clang-format`
- 建议启用 pre-commit（提交时自动格式化/检查）：
```bash
pip install pre-commit
pre-commit install
```

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
- 配置文件()
- 代码规范(py and cpp)

- log(hydra)

- 三轮车的模型好像有问题
- 场景自主生成(simple的进来和实际的对不上，main是固定的， 想要直接就能生成+打开一套，不弄那么负责)
- 数据采集()
- 需要有一个可视化的界面，把joystick啥的都，
- 最基本的utilize的脚本，rosbag 的记录和显示的脚本，想
-

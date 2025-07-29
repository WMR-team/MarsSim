## 自定义参数

- `bg_path`： 背景地形图路径，需要是深度图，像素长宽度需要是 640x640，数据类型为 uint16，默认路径在`"MarsSim_v2_ws/src/rover_gazebo/models/mars_terrain/heightmaps_int16/HM5.png"`，具体路径请根据具体使用的地图赋值。


---

## 陨石坑信息保存
运行`crater_gen.py`后，会开始随机大小和形状得生成陨石坑，同时在同级目录下生成 `crater_info.csv`，其中标记生成的陨石坑信息的真值，包含如下数值。


| 参数 | 含义 |
|------|------|
| `crater_x` | 陨石坑中心 x 坐标 |
| `crater_y` | 陨石坑中心 y 坐标 |
| `D` | 陨石坑直径 |
| `alpha_c` | 陨石坑坑腔内壁的幂律指数 |
| `ratio` | 陨石坑坑内和坑缘长度的比例（用于控制陨石坑坑缘的圆钝度） |


---

## 陨石坑生成过程

运行`crater_gen.py`后，会开始随机大小和形状得生成陨石坑，会出现open3d的窗口用于检查，其中可视化陨石坑的三维曲面形态；关闭后会出现matplotlib的窗口，可视化陨石坑的二维深度图，同时在终端打印陨石坑的信息用于检查；在关闭matplotlib窗口后，需要在终端输入选择，分别如下：

- `s`: 保存当前陨石坑，将把陨石坑加入到背景地形图中，保存信息；

- `c`: 不保存陨石坑，继续随机生成；

- 其他键：退出程序，生成结束。

---

## 输出含陨石坑的地图

在MarsSim生成地形的深度图目录下会生成两张png图片，分别如下：

- 加入陨石坑的uint16深度图，长宽为640x640

- 缩放后的uint8深度图，长宽为513x513



---

## 更新MarsSim 地形

- 将生成的640x640的uint16深度图默认`MarsSim_v2_ws/src/rover_gazebo/models/mars_terrain/heightmaps_int16/`下，命名为原地图的名字，如`HM5_crater.png`。

- 将生成的513x513的uint8深度图替换到`MarsSim_v2_ws/src/rover_gazebo/models/mars_terrain/heightmaps_int8/`下，命名为原地图的名称，如`HM5_crater.png`。

- 删除 home目录下的 .gazebo 文件夹中的paging文件夹

- 修改 `MarsSim_v2_ws/src/world_plugins/scripts/world_change_pipeline.py` change_world 函数的 `heightmap_num` 参数，例如从 5 改成 "5_crater"

- 运行 `MarsSim_v2_ws/src/world_plugins/scripts/world_change_pipeline.py` 更新地图
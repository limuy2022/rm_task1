# Gazebo 仿真资源

本包安装 Gazebo Fortress 的场景、网格模型和 Python 辅助模块。
运行入口仍为仓库 `tools/simulate_gazebo_lio.py` 与 `tools/simulate_gazebo_vision.py`；
完整环境与验收命令见[雷达仿真教程](../../docs/教程/Gazebo雷达惯性定位仿真.md)。

## 资源与路径

- `worlds/lio_room.sdf`：可人工编辑的 Blender 场景。
- `worlds/lio_room_regression.sdf`：标准房间，供定位和视觉联合回归使用。
- `meshes/scene.stl`：运行时模型，随包安装。
- `blender/`：保留 Blender 源文件与既有备份，不安装进运行资源目录。
- `team43_gazebo/`：传感器参数、输入审核和退出收尾辅助模块。
- `plugins/`：独立编译的 C++ 施力插件源码，不参与普通 colcon 构建。

`worlds/` 和 `meshes/` 安装到 `share/team43_gazebo/`。
场景模型统一引用 `package://team43_gazebo/meshes/scene.stl`，碰撞体与视觉体一致。
source 工作区 `install/setup.bash` 后，环境 hook 将当前安装前缀的 `share/`
加入 `IGN_GAZEBO_RESOURCE_PATH`；Fortress 去掉 URI 协议前缀后据此查找资源。
该路径来自安装前缀，不依赖源码目录、当前工作目录或生成 SDF 的输出目录。

脚本通过 ament 索引读取本包场景与定位包配置。`RM_LIO_WORLD` 可覆盖源 SDF 路径；
未设置时使用安装目录的 `worlds/lio_room.sdf`。标准回归在 source 后设置：

```bash
export RM_LIO_WORLD="$(ros2 pkg prefix --share team43_gazebo)/worlds/lio_room_regression.sdf"
```

## 可选插件

根 Dockerfile 的 `gazebo-sim` 目标单独编译 `plugins/`，安装到
`/opt/team43_gazebo/lib/`，并配置 `IGN_GAZEBO_SYSTEM_PLUGIN_PATH`。
SDF 只引用插件库名 `librm_task1_force_controller.so`，不写死安装绝对路径。
普通测试镜像只需构建本包的资源与 Python 模块，不额外依赖 Gazebo 开发库。

本包只服务仿真，不访问串口或真实电机，不修改实车运动开关。

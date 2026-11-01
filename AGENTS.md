# AGENTS.md

RoboMaster 赛题一：云台视觉跟踪 + 底盘定位 + 评分提交链。链路为 ArUco 识别 → PnP → P/PI/PD/PID 控制 → C++ 串口桥下发两轴角速度；另有 Point-LIO 底盘定位、点云建图与九列 CSV 评分提交。ROS 2 Humble，Python 包为主。

## 目录结构

- `src/` — 16 个 ROS2 包，统一命名为 `team43_<功能>`，源码目录、ROS 包名和 Python 模块名保持一致。12 个 Python 包（ament setup.py 布局）+ `team43_interfaces`、`team43_serial_bridge`、`team43_lio_backend`、`team43_gazebo` 四个 ament_cmake 包；`team43_serial_bridge` 为 C++ 串口桥（gtest），`team43_gimbal_io` 为 Python 仿真串口收发与硬件消息适配。`team43_gazebo` 安装仿真资源与 Python 辅助模块，C++ 运动插件仅在可选 Gazebo 镜像中编译。
- `tools/` — 离线验证与评分脚本；`verify_all_offline.py` 是全量离线核验编排入口。
- `docs/` — `教程/`（含测试、标定、上机步骤）、`原理/`、`记录/`；改动相关模块前先读对应教程。
- `gimbal-edu-tools/` — git 子模块，`test_consistency` 校验其中 `virtual_gimbal` 的 entry_points，克隆后必须检出（`git submodule update --init --recursive`）。
- `build/offline/` — 验证日志与合成产物，已被 gitignore，勿提交。

## 如何运行测试（首要规则）

本机不装 ROS；所有验证在与 NUC 同源的 Docker 容器（根 `Dockerfile`）中运行：

- **本地有 Docker** → 在容器内跑测试：

  ```bash
  docker build -t rm-task1-humble-test .
  docker run --rm -v "$PWD":/workspace -w /workspace \
    --user "$(id -u):$(id -g)" -e HOME=/tmp rm-task1-humble-test bash -c '
      source /opt/ros/humble/setup.bash &&
      colcon --log-base /tmp/clog build --build-base /tmp/cb \
             --install-base /tmp/ci --symlink-install &&
      source /tmp/ci/setup.bash &&
      colcon --log-base /tmp/clog test --build-base /tmp/cb \
             --install-base /tmp/ci --packages-select team43_serial_bridge team43_lio_backend &&
      colcon --log-base /tmp/clog test-result --test-result-base /tmp/cb --verbose &&
      python3 -B -m pytest -q src --import-mode=importlib &&
      python3 -B tools/verify_all_offline.py --skip-pytest'
  ```

- **本地没有 Docker** → 提交并推送，关注云端 GitHub Actions（`.github/workflows/ci.yml`）的结果（如 `gh run watch` 或 Actions 页面）；失败时下载 `offline-verification` artifact 查看 verify 日志与 result.json。

- 聚焦测试（交互式容器内：`docker run -it --rm -v "$PWD":/workspace -w /workspace --user "$(id -u):$(id -g)" -e HOME=/tmp rm-task1-humble-test bash`，先跑上面的 colcon build 并 `source /tmp/ci/setup.bash`）：
  - 只跑 pytest：`python3 -B -m pytest -q src --import-mode=importlib`
  - 单文件/单用例：`python3 -B -m pytest -q <测试文件>::<test_name> -v`
  - C++ gtest：`colcon --log-base /tmp/clog test --build-base /tmp/cb --install-base /tmp/ci --packages-select team43_serial_bridge team43_lio_backend` + `test-result`
  - 瞄准鲁棒性快速档：`python3 -B tools/verify_aiming_robustness.py --quick`
- 速查表与分层说明见 `docs/教程/测试运行指南.md`。

## 硬性约束

- **绝不用 pip 覆盖 ROS 环境的 OpenCV/numpy**（镜像与 NUC 均为 apt OpenCV 4.5.4）；镜像里也刻意不装 pip。
- 容器内仓库挂载到 ASCII 路径 `/workspace`（宿主路径含中文）；colcon 的 build/install/log 全放 `/tmp`，不在仓库内落构建产物；`--user $(id -u):$(id -g)` 避免产生 root 属主文件。
- 测试环境的单一来源是根 `Dockerfile`，CI 跑的是与本地逐字相同的命令——本地全绿 CI 即全绿；改环境只改 Dockerfile，不在 CI 里另建环境。
- 公共 NUC 红线：不用 pip 覆盖系统库、不全量构建、不改 `.bashrc`、不读主办方保护目录、不向 `/judge/*` 发布测试消息。
- 保留既有源码的版权与许可证声明。

## 约定

- 文档、注释与 commit message 用中文；commit 走 conventional 前缀（`ci:` / `refactor:` / `test:` / `fix:` 等）。
- 台架与独立节点保持运动默认关闭；按用户比赛操作要求，`start_match.bash`/`match.launch.py`直接启动全链并默认`motion_enabled=true`、`continuous=true`。只读采样和标定必须显式传`motion_enabled:=false`，故障/反馈/限位保护继续生效。

比赛流程：先运行`tools/start_match.bash`（默认target_id=-1），裁判给ID后运行`tools/set_target.bash ID`，确认跟踪后裁判启动客户端。比赛入口不使用match_session_gate，不等待confirmed开关或预填epoch匹配；沿用文件中的几何和旋转数值，首次反馈绑定当前epoch，运行中复位仍锁停。

评分起点：校内赛环境说明保证赛题一第0拍同时发布`/judge/replay_start`（Header、transient_local、frame_id为record或replay）。以本轮有效replay_start的stamp冻结原点，不用tick替代。保留旧锁存起点过滤，并允许tick与replay_start以任意回调顺序到达；CSV仍逐tick立即写行。

本轮vehicle_field.yaml已由data/现场外参记录.md生成用户授权的近似配置：镜头前端/银圈代理传感器原点，固定轴向采用理想模型。该配置开启kinematics.align_world_at_start；启动前底盘静止、相机水平朝车头，首组同步姿态自动求世界对齐，运行中复位保护继续生效。start_match.bash固定默认加载本轮vehicle_field.yaml，其他配置通过vehicle_config显式指定；不要把近似标成独立精度验收通过。

当前走廊比赛方案（2026-11-01）：用户确认地图不参与评分、建议速度仅输出，不依赖地图规划；随后要求建议应考虑撞墙。start_match/match.launch默认map_mode=copy，从maps/43_map.pgm/yaml复制交付文件，不读取maps/current，不启动saved_map_node或等待跨会话重定位。LIO、雷达到底盘中心外参、云台世界对齐、正式replay_start原点及逐tick CSV保持。运行快照设chassis.dir_mode=direct、enable_avoidance=false；缺图不阻塞建议。direct模式先算目标方向，再以实时cloud_registered（lio_world→map的实际TF）作障碍保护：启用obstacle_guard_enabled，地面沿用LIO近似，底盘长宽按用户20～30cm的上限近似，中心距障碍0.4m并计入点云离散误差。前瞻1.5秒、建议制动模型0.5m/s²，必要时限速/转向/归零；速度下限不得覆盖保护结果。点云0.5秒过期、坏包、无TF或地面无效时建议归零，但CSV继续；底盘/目标/观测过期与丢失、近距离停止、限速保持。显式corridor实验模式的corridor_map_node只用当前点云拟合平行墙生成可选示意图，不修改LIO、不参与建议；失败或退出不得关闭评分链。上午两图宽度估计2.500/2.487m，安装corridor_model.yaml用中值约2.494m筛选候选，不能标为实测验收。模型40m纵向范围仅是显示窗口，不是门间长度，不虚构端墙。

map_mode=none可完全关闭地图输出；saved/live保留作显式实验模式。saved模式必须显式传map_yaml，运行快照仍设chassis.map_timeout_s=0.0；不通过重发伪造源stamp。旧图重定位测试须显式指定saved和地图路径，不得依赖比赛默认值。上午start_mapping/save_mapping及record仍可独立采集，既有原件保留。

原始数据：tools/record.bash是只读ROS bag录制入口，默认完整点云/雷达IMU，--with-camera额外相机/云台；不会启动驱动或运动。必须在原始数据中保留5～10秒静止供离线初始化，结束等待Ctrl+C收尾和SQLite/metadata计数校验。verify_record已纳入verify_all_offline（20项），跨会话长测使用独立评分源match_tick_source和独立CSV观察器，不让大图像与评分共用DDS发送进程；时限不放宽。

Gazebo仅保留src/team43_gazebo/worlds/corridor.sdf（8×2.5m测试走廊，非实测长度），删除原房间、Blender与STL模型。主要回归为tools/simulate_gazebo_corridor.py，CI的gazebo_lio运行此工具；视觉仿真也只以走廊为基础添加相机和目标码，不增加箱体场景。墙宽、防撞与LIO精度分项判断，任何必需判据失败均不得称全绿。

当前地图交付方案：用户选定第一张data/survey_20261101_105202_506921，maps只保留43_map.pgm和43_map.yaml，image改为43_map.pgm，PGM像素、分辨率和上午origin保持。start_match/match.launch默认map_mode=copy，从本队maps复制到正式output，保留源坐标，不声称变成下午map。复制失败仅提示，不阻塞CSV。copy模式processing with_map=false、judge.map_write_enabled=false，停用运行地图生成/转换/写出，避免覆盖复制文件。LIO、replay_start、云台世界对齐和实时防撞保持；copy/corridor/none均使用direct加实时障碍保护。maps的这两个文件纳入源码交付，部署归档不得排除maps。corridor/saved/live仅作显式实验模式。

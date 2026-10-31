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
- 台架与独立节点保持运动默认关闭；按用户比赛操作要求，`start_match.bash`/`match.launch.py`在标定及源图就绪后默认`motion_enabled=true`、`continuous=true`。只读采样和标定必须显式传`motion_enabled:=false`，故障/反馈/限位保护继续生效。

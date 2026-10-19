# gimbal_serial

教学云台套件的串口桥，课 1 到课 3 和固件联调使用。改自 ling_vision 的 robo-bridge，通信契约见 `protocol/PROTOCOL.md`（v1.4），与本包冲突时以契约为准。

## 做什么

- 从串口收 0x01 帧（IMU，500Hz），发布 `/red_standard_robot1/gimbal`（`sensor_msgs/Imu`，赛题一要求的话题名，可用 `imu_topic` 参数改）和 TF `world -> gimbal_link`。
- 可选：订阅 `/offboardlink/cmd_vel_gimbal`（`geometry_msgs/Twist`，可用 `cmd_vel_topic` 参数改），把 `angular.z`（yaw）和 `angular.y`（pitch）当作关节角速度，每收到一条消息转发一帧 0x30。不做保活，持续跟踪要自己持续发布话题。
- 本副本供赛题一使用，是从 `gimbal-edu-tools/robo_bridge_edu` 复制后改名（包名 `gimbal_serial_42`，目录 `42-gimbal_serial`，位于仓库 `src/` 下），**不包含**海康相机驱动源码（`hik_camera_node`），相机由主办方启动，见下面的“海康相机（可选）”。
- 同一时刻只能有一个进程打开串口。

## 使用

```bash
# 真机，port 为 auto 时依次尝试 RM_SIM_SERIAL，/dev/gimbal，第一个 /dev/ttyUSB* 或 /dev/ttyACM*
ros2 launch gimbal_serial_42 gimbal_serial.launch.py
# 虚拟云台
ros2 launch gimbal_serial_42 gimbal_serial.launch.py port:=/tmp/ttyVGIMBAL
# 兼容旧脚本的环境变量写法
RM_SIM_SERIAL=/tmp/ttyVGIMBAL ros2 launch gimbal_serial_42 gimbal_serial.launch.py
# 打开 cmd_vel_gimbal 转发
ros2 launch gimbal_serial_42 gimbal_serial.launch.py subscribe_cmd_vel:=true
```

launch 参数留空就沿用 `config/bridge.yaml`。串口打不开时节点直接报错退出，launch 默认不重启（`respawn:=false`）。`respawn:=true` 只在设备文件还不存在（节点退出码 3）时每秒重试一次；端口被占用（EBUSY、EWOULDBLOCK）或参数非法（退出码 1）一律不重试，避免抢走别的节点刚释放的串口。

## 赛题一对接

- IMU 话题就是比赛要求的 `/red_standard_robot1/gimbal`（默认值），无需额外配置。
- 控制云台：`subscribe_cmd_vel:=true` 后向 `cmd_vel_topic`（默认 `/offboardlink/cmd_vel_gimbal`）发布 `Twist`，`angular.z` 为 yaw、`angular.y` 为 pitch 关节角速度（rad/s）。下位机看门狗 0.5 s，**必须 ≥20 Hz 持续发布**（丢目标、退出时发零指令）；限幅 2 rad/s、斜率限制 15 rad/s²，PC 侧也应先限幅。上电后要等收到第一帧 0x01（IMU 就绪）才能发指令，之前会被拒收。
- 方向验证：正 `angular.z` 应使 IMU 的 `angular_velocity.z > 0`，正 `angular.y` 应使 `angular_velocity.y > 0`；不对时先查 `yaw_sign`/`pitch_sign`。
- 定位栈自建 TF 时用 `publish_tf:=false` 关掉本节点的 `world -> gimbal_link`，避免 `gimbal_link` 出现两个父节点。
- 比赛机串口设备为 `/dev/ttyUSB*`/`/dev/ttyACM*`（权限 777），`port: auto` 会自动枚举；`/dev/gimbal` 的 udev 规则不依赖，装不装都行。

## 选口诊断

`/dev/gimbal` 由 `setup/99-gimbal.rules` 创建，规则按芯片 VID:PID 匹配（CP210x `10c4:ea60`、CH340 `1a86:7523`）。规则注释自己写明：同一台机器插了两颗同型号适配器时两个都匹配，`/dev/gimbal` 指向**后插的那颗**——桥打开成功不代表打开的是云台。为此桥在打开端口后打印三类诊断，全部只提示、不改变任何行为：

1. 启动日志解析真实身份：`串口 /dev/gimbal（来源 udev）-> /dev/ttyUSB2（by-id: usb-Silicon_Labs_CP2102_...）`。PTY 等没有 by-id 条口的设备打印 `（无 by-id 条目，可能是 PTY）`。
2. 机器上出现 ≥2 颗 CP210x/CH340 类适配器时告警一次，列出全部 by-id 名，并提醒用 `port:=/dev/serial/by-id/<名字>` 显式指定。
3. 超过 `imu_wait_warn_s` 仍无 0x01 时按累计字节数分支：**0 字节**提示"指向错误适配器或物理无数据（供电/TX-RX 未交叉/未共地），也可能刚上电在温和标定"；**有字节无帧**提示"优先核对波特率 921600 与 TX/RX 接线"。实机排查命令：`lsusb`、`ls -l /dev/serial/by-id/`、`ls -l /dev/gimbal`。

## 参数

- `port`（默认 `auto`）：显式写路径时原样使用，此时 `RM_SIM_SERIAL` 被忽略，日志里会提醒。实机建议直接写 `/dev/serial/by-id/<云台适配器的名字>`，见上面的“选口诊断”。
- `baudrate`（921600）：8N1，无流控，raw 模式。
- `imu_topic`（`/red_standard_robot1/gimbal`）：IMU 发布话题，赛题一要求的名字。
- `imu_axis_sign`（`[1, 1, 1]`）：只作用于 accel 和 gyro，不作用于 q。每项必须是 1 或 -1。
- `publish_tf`（true）：TF 只能由 IMU 一方发布。
- `subscribe_cmd_vel`（false），`cmd_vel_topic`，`yaw_sign`，`pitch_sign`（都是 1）：发送前乘的符号，见契约第 8 节。
- `imu_wait_warn_s`（15）：这么久没有第一帧 0x01 就提示一次，按累计字节数分“零字节/有字节无帧”两种话术（见“选口诊断”）。只提示，不退出，因为固件上电后要等到温和标定完成才发 0x01。

## 发布内容

- `header.stamp` 是主机接收时刻，帧通过校验之后立刻取 `node->now()`，Imu 和 TF 用同一个值，不加偏移。stamp_us 只用来排序和求 dt，不映射成 ROS 时间。
- QoS：可靠，volatile，keep_last 200。
- accel 是比力（`raw * 0.00718260498046875`），静止水平时约为 (0, 0, +9.8)。gyro 是 `raw * 0.00106526443603169529841533860381`。两者三轴都不取负。
- orientation 是归一化后的 q（协议顺序 wxyz，放进 ROS 消息的 xyzw 字段）。q 范数小于 0.5 的帧丢弃并计数。
- 协方差：orientation 对角 (1e-4, 1e-4, 1e-2)，angular_velocity 对角 1e-5，linear_acceleration 对角 1e-2。
- 检测到 MCU 复位（stamp 倒退，或 seq 回退不少于100）时打一条日志，这一帧不计丢帧，照常发布。累计的校验错，应用层丢弃，复位和丢帧数在有变化时每秒最多打一条。

## 海康相机（可选）

> **本副本不包含海康驱动的源码**：`src/hik_camera_node.cpp`、`test/hik_sdk_test.cpp` 均未从 `robo_bridge_edu` 复制过来，因此 `WITH_HIK_CAMERA` 默认为 `OFF`；即使改成 `AUTO`，源码缺失时也会跳过（`ON` 则直接报错），不会在装有 MVS SDK 的机器（比赛机）上编译失败。赛题一的相机由主办方启动（`robo_bridge_edu hik_camera.launch.py`），本包不需要相机节点。需要自带相机时，从 `gimbal-edu-tools/robo_bridge_edu` 复制上述源文件后再显式 `-DWITH_HIK_CAMERA=ON`。以下内容为原版 README 留档。

原包里的驱动发布 `/image_raw`（`sensor_msgs/Image`，rgb8）和 `/camera_info`，`frame_id` 固定为 `camera_optical_frame`，与 `gimbal_edu_description` 一致。只枚举 USB 相机，打开枚举到的第一台。

### 为什么改用系统 SDK

原来的驱动把 SDK 的 `.so` 放在包里，还装进 `install/lib`，那个目录在 ROS 的 `LD_LIBRARY_PATH` 上，会遮住系统里的 SDK。相机固件升级、需要新 SDK 时，就得手工替换包里的库再重新编译。现在：

- 包里没有任何海康的头文件和 `.so`，固件升级后只需要重装新版 MVS 的 deb。
- MVS 的库 SONAME 不带版本号（`libMvCameraControl.so`），组件库 `libgimbal_serial_hik_camera.so` 里记的 NEEDED 就是这个名字（`hik_camera_node` 可执行文件本身不链接 SDK），RUNPATH 指向稳定路径 `/opt/MVS/lib/64`。所以重装 MVS 之后，不重新编译也会加载新库。
- 新版 SDK 的头文件可能有变化，建议升级 MVS 后重新编译一次。
- 只链接 `libMvCameraControl.so`，`libMvUsb3vTL.so` 等其他库由 SDK 自己加载，不需要设 `LD_LIBRARY_PATH`。

### 安装

```bash
sudo dpkg -i MVS-*.deb
# 安装脚本会往 /etc/profile 和 ~/.bashrc 写 MVCAM_SDK_PATH 等变量，开新终端或者 source 一下
source /etc/profile
```

除 SDK 之外还要装 `ros-humble-camera-info-manager` 和 `ros-humble-image-transport`（`setup/deps.md` 里的包列表已含后者），以及运行时要用的 `libusb-1.0-0`。

### 编译和确认

```bash
colcon build --symlink-install --packages-select gimbal_serial_42
# 配置输出里应有 "hik_camera: enabled, MVS SDK /opt/MVS/lib/64/libMvCameraControl.so -> ..."
ros2 pkg executables gimbal_serial_42      # 应该有 hik_camera_node
```

CMake 选项：

- `WITH_HIK_CAMERA`：`AUTO`（默认）SDK 和 ROS 依赖都齐全才编译，缺任何一项就跳过，只在配置输出里写明原因；`ON` 缺任何一项直接报错，写明找的是哪个路径；`OFF` 不编译。
- `MVS_ROOT`：SDK 安装目录，默认取环境变量 `MVCAM_SDK_PATH`（目录存在时），否则 `/opt/MVS`。头文件找 `${MVS_ROOT}/include`，库找 `${MVS_ROOT}/lib/64`（aarch64 是 `lib/aarch64`，armv7 是 `lib/armhf`）。改了 `MVS_ROOT` 之后，缓存的查找结果会自动失效。

```bash
colcon build --packages-select gimbal_serial_42 --cmake-args -DWITH_HIK_CAMERA=ON
colcon build --packages-select gimbal_serial_42 --cmake-args -DMVS_ROOT=/opt/MVS_other
```

### 启动

```bash
ros2 launch gimbal_serial_42 hik_camera.launch.py
# 指定内参文件
ros2 launch gimbal_serial_42 hik_camera.launch.py camera_info_url:=package://camera_setup/config/camera_info_640x480.yaml
# 换参数文件，取图失败退出后自动重启
ros2 launch gimbal_serial_42 hik_camera.launch.py params_file:=/path/to/my_hik.yaml respawn:=true
```

launch 参数：`params_file`（默认 `config/hik_camera.yaml`），`camera_info_url`（留空沿用 yaml），`respawn`（默认 false）。如果这台机器上 AUTO 跳过了编译，launch 会报找不到可执行文件 `hik_camera_node`，先按上一节确认。

启动时节点会打印 SDK 版本和实际加载的库路径，固件或 SDK 不匹配时先看这两行：

```
MVS SDK version: 4.8.2.2
MVS SDK library: /opt/MVS/lib/64/libMvCameraControl.so (-> /opt/MVS/lib/64/libMvCameraControl.so.4.8.2.1)
```

参数（`config/hik_camera.yaml`）：

- `camera_name`（`hik_camera`）：camera_info_manager 用的相机名。
- `camera_info_url`（空）：内参文件的 URL。留空用 camera_info_manager 的标准位置 `~/.ros/camera_info/<camera_name>.yaml`。海康相机的输出分辨率通常不是 640x480，`camera_setup` 里的默认内参不适用，需要按 `camera_setup/CALIBRATION.md` 标定。
- `exposure_time`（5000，整数）：曝光时间，微秒，范围取自相机。
- `gain`（15.0）：增益，范围取自相机。
- `mfs_path`（空）：相机特征文件（`.mfs`）。非空时加载这个文件；为空且设了环境变量 `RM_PATH` 时加载 `${RM_PATH}/config/camera/camera_1080p.mfs`（ling_vision 真机的做法）；两者都没有就不加载，沿用相机当前设置。加载失败（返回码非 0）只打 WARN，不退出。

### 注意事项

- USB 权限（udev 规则）由 MVS 的安装脚本负责，本包不管。装完之后重新插拔相机；普通用户打不开相机时先查这一条。这一条没有在实机上验证。
- 相机同时只能被一个进程打开。MVS 自带的客户端开着时，节点会打不开相机。
- 运行时要系统装有 `libusb-1.0-0`。
- WSL 里没有 USB 相机（需要 usbipd 才能直通），只能验证到“枚举出 0 台”：SDK 版本、库路径、`Found camera count = 0`、每秒一条 `No camera found!`，Ctrl-C 可以正常退出。取图、参数范围、`camera_info` 都没有在 WSL 里验证过。
- 没有相机时构造函数一直卡在枚举循环里，节点不会 spin，所以 `ros2 param get`、话题这些都查不到；这是原版的行为。
- 在没有 `DISPLAY` 的干净环境里，SDK 自己会在标准输出打印 `XOpenDisplay Fail`，无害。
- SDK 会在进程的当前目录下建 `MvSdkLog/` 写日志（在 WSL 里实测是这样）。不要在源码目录里直接运行节点，从工作区根目录或别的目录启动，免得把日志带进源码。

### 已知问题

以下来自 ling_vision 原版，这次没有修，行为保持不变：

- 第一帧：`image_msg_.data` 只做了 `reserve`，`size()` 是 0，所以第一次像素转换的目标缓冲区大小是 0，第一帧图像不可信。
- 取图失败（1 秒超时，例如拔掉相机）时节点直接 `exit(1)`，不会重连。需要自动恢复就用 `respawn:=true`。
- 只枚举 USB 相机，不支持 GigE；多台相机时只打开第一台。
- 第一次打开设备并加载 mfs 之后，会做一轮 StartGrabbing、StopGrabbing、Close、Destroy，再重新 Create、Open，保持原版不变。

### 测试

`gimbal_serial_hik_sdk_test` 只在编译了海康驱动时才加进构建，不需要相机，5 个用例：

- `VersionNonZero`：`MV_CC_GetSDKVersion()` 不为 0。
- `LoadedFromSystemSdkDir`：`dladdr` 取到实际加载的库，它所在的目录必须就是编译时的 `MVS_LIB_DIR`，用来证明加载的是系统 SDK，不是哪份副本。
- `InitEnumFinalize`：`MV_CC_Initialize`，`MV_CC_EnumDevices(MV_USB_DEVICE)`，`MV_CC_Finalize` 都返回 `MV_OK`，设备数可以是 0。
- `ComponentLibraryLoads`：`dlopen` 组件库必须成功。
- `ComponentLibraryResolvesSdkByOwnRunpath`：测试进程本身已经链接了 SDK，上一条看不出组件库自己能不能解析，所以在清空环境变量的子进程里对组件库跑 `ldd`，要求 `libMvCameraControl.so` 解析到 `MVS_LIB_DIR`。

`LD_LIBRARY_PATH` 的优先级比 RUNPATH 高。MVS 安装脚本会把 `/opt/MVS/lib/64` 写进 `LD_LIBRARY_PATH`，平时它和 RUNPATH 是同一个目录，不影响结果。如果用 `-DMVS_ROOT` 指向另一套 SDK 编译，就要在 `env -i` 这类干净环境里跑测试，否则 `LoadedFromSystemSdkDir` 会因为加载了 `/opt/MVS` 那一套而失败。

## 与 ling_vision 原版的差别

- 包名 `robo_bridge` 改为 `gimbal_serial`；删除 AutoAimer 等消息，以及对 robo_utils，auto_aim_interfaces 的依赖。
- 帧解析改用契约 3.4 节的状态机（所有分支下标有界，校验失败整帧丢弃），原来的 OffboardLink 解析器和全局 `bridge` 单例不再使用。
- 串口从 CSerialPort 换成 `serial_port.hpp` 里的 termios 封装：`open`，`TIOCEXCL`，`flock(LOCK_EX|LOCK_NB)`，任何一步失败直接报错。CSerialPort 没有暴露 fd，加不了独占。
- accel 和 gyro 不再取负；删除 `imu_payload_mode`，WorldTranslation 分支和 `decodeImuPayloadWords` 的分模式逻辑，固定按比力解码。
- TF 父系 `odom` 改为 `world`，平移恒为 0，去掉 `camera_imu_stamp_offset`。
- 节点由 lifecycle 节点改为普通节点。
- 海康相机驱动（ling_vision 的 `ros2_hik_camera`，src@06f74f4）并入本包，成为可选的 `hik_camera_node`：不再带 `hikSDK/` 里的头文件和 `.so`，也不再把 SDK 的库装进 `install/lib`，改为链接系统 MVS SDK 的 `libMvCameraControl.so`，RUNPATH 指向 `/opt/MVS/lib/64`；没有 SDK 时自动跳过。
- 驱动源码只改了几处：启动时打印 SDK 版本和实际加载的库路径；新增参数 `mfs_path`，去掉写死的 `/home/dut0bug/...` 路径（`RM_PATH` 的做法保留）；`camera_info_url` 默认值从指向 `rm_vision_bringup` 的 `package://` 改为空；没有相机时收到退出信号不再对未初始化的设备信息调用 CreateHandle，`camera_handle_` 初始化为 `nullptr`。
- 原 `declareParameters()` 里的 `MV_CC_FeatureLoad(camera_handle_, mfs_path_.c_str())` 删掉：`mfs_path_` 一直是空串，那一行什么也没做。

## 与 gimbal-edu-tools/robo_bridge_edu 的差别

- 目录与包名：`src/42-gimbal_serial` / `gimbal_serial_42`（赛题要求 `~/ws_championship/src/<队伍编号>-<名称>`），命名空间 `gimbal_serial`，可执行文件仍为 `robo_bridge_node`。
- IMU 话题：从硬编码 `/offboardlink/imu_data` 改为参数 `imu_topic`，默认 `/red_standard_robot1/gimbal`（赛题一要求）。
- 不包含海康相机驱动源码（`hik_camera_node.cpp`、`hik_sdk_test.cpp`），`WITH_HIK_CAMERA` 默认 `OFF`，AUTO 模式下源码缺失也会跳过；赛题一相机由主办方启动。
- launch 文件名为 `gimbal_serial.launch.py`。
- 协议层（帧解析、IMU 换算、TF、串口封装）与原版逐行一致，未做任何改动。

## 测试

```bash
colcon test --packages-select gimbal_serial_42 && colcon test-result --verbose
```

- `protocol_test`：读 `protocol/golden_frames.txt`（包内不放副本），全部 accept、reject 用例逐帧走 `FrameParser`，`decodeImu` 和 `makeImuMsg`，再加流式测试，物理分组检查，复位检测。找不到黄金向量时 cmake 只给 WARNING，build 不会失败，但依赖黄金向量的用例会被跳过（跳过不等于通过）；设环境变量 `GOLDEN_FRAMES_DIR` 指向 `protocol/` 目录再运行 cmake 即可恢复。
- `serial_port_test`：pty 上的独占打开，读写，对端关闭。`close()` 先 `ioctl(TIOCNXCL)` 再 `close`（否则 pty 的 master 还开着时，独占标志一直留着，非 root 重开会 EBUSY）。root 会绕过 TIOCEXCL，root 下测试打印 “root 下未覆盖独占”，改用 `TIOCGEXCL` 直接读标志位；非 root 的行为要用非 root 用户再跑一遍（`setpriv --reuid=65534 --regid=65534 --clear-groups <测试可执行文件>`）。
- `node_test`：pty 当串口，节点与测试节点同进程，检查发布内容，cmd_vel 转发，退出时补零指令。
- `hik_sdk_test`：只在编译了海康驱动时才有，不需要相机，检查加载的是系统 SDK、SDK 能初始化和枚举、组件库能被加载，见“海康相机（可选）”一节的“测试”。

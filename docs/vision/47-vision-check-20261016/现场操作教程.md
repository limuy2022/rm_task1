# 一小时实机检查：Windows传文件 → NUC编译 → 视觉和传感器检查

更新：2026-10-16。本队编号暂用47。

**操作环境已按实际情况写定：B在自己的Windows上打开NoMachine，连接NUC的Ubuntu。本文所有Linux命令都输入NoMachine远程桌面里的终端，不输入Windows的PowerShell，也不用切换到B电脑的Linux。**

相机、雷达、云台和NUC已经启动，直接检查它们现有的数据。今天实际运行本队的ArUco检测和位置解算，确认真实输入，再量相机与IMU的安装关系。

## 一、先认识今天会用到的几个词

### 1. 串口是什么？现在需要操作它吗？

串口是NUC与云台控制板传数据的一条通信通道。实物上通常经USB线或USB转接器连接；Linux给它一个设备名，例如`/dev/ttyUSB0`。控制板把IMU数据沿这条通道送给NUC，桥接程序再把数据变成ROS话题。

```text
云台控制板 ──串口连接──> 已运行的桥接程序 ──> 云台IMU话题
相机       ──相机驱动────────────────────> /image_raw
雷达       ──雷达驱动────────────────────> 点云话题和雷达IMU话题
```

**设备上电不一定意味着桥接程序已经发布云台IMU。今天先检查话题；有反馈就直接使用，没有就记下具体缺项。** 视觉测距本身不需要打开串口。本教程不另开串口桥，不发云台运动命令。

对应资料：[主办方串口桥说明](https://blog.secaumene.com/kit/bridge.html)、[统一通信协议](https://blog.secaumene.com/kit/protocol.html)。看“IMU数据”和“串口”部分，用来理解数据从哪里来。

### 2. 机械零位是什么？限位是什么？

- **yaw**：云台左右转向的那根轴。
- **pitch**：相机抬头、低头的那根轴。
- **机械零位**：给关节角规定一个能重复找到的起始姿态。例如“相机沿底盘车头看、镜头水平”可以作为一种约定，但必须与控制系统实际定义一致。贴线、定位孔、编码器归零方法或机械图可以帮助重复找到它。
- **限位**：每根轴允许转动的范围。超出可能撞结构、拉扯线缆；软件通常会在机械边界以内设置更保守的范围。

今天C拍下定位标记、转轴和线缆走向，记录现有说明中的零位/允许范围；没有明确依据就写“未确定”。**不用通过转到头、碰撞或强扳电机来找限位。** IMU显示yaw=0，也不能据此说镜头正对车头；IMU参考朝向和机械关节零位是两件事。

对应资料：[第一课的坐标系与姿态](https://blog.secaumene.com/teach/class-1.html)、[统一协议里的姿态定义](https://blog.secaumene.com/kit/protocol.html)。今天看懂区别即可，零位/限位不会阻塞下面的纯视觉检查。

## 二、这一小时怎么安排

**传文件最好提前完成。** 若只能到场后传，就按下表执行；编译超过8分钟仍未完成时，先让它保留日志，A/C继续测量，B先检查已有传感器数据。

| 时间 | B在电脑上做 | A在实物旁做 | C在实物旁做 |
|---|---|---|---|
| 0—15分钟 | 传压缩包、解压到专用目录、只编译四个视觉相关包 | 找相机光轴、准备照片和测量基准 | 拍相机/云台/IMU装配，量ArUco黑色外边长 |
| 15—25分钟 | 按第六、七部分检查相机、雷达和两种IMU | 与C确认三个坐标的方向 | 协助标轴，记录零位标记和安装状态 |
| 25—45分钟 | 按第八部分运行本队视觉；录近、中、远三档结果 | 量目标实际位置、记录测距基准 | 移动目标码，完成右、下、远、遮挡动作 |
| 45—53分钟 | 按第九部分量相机—IMU关系；有余量再读已有LIO | 主持三个分量的测量 | 配合尺量和拍照 |
| 53—60分钟 | 检查bag，打包下载结果 | 写本次结论、保留未知项 | 核对照片与测量位置一一对应 |

只用三个NUC终端：**终端1运行视觉，终端2输入检查命令，终端3录数据。** B操作键盘。A/C交替摆码、量尺、记录。

## 三、先把哪些文件传到NUC，放哪里？

### 步骤1：A把传输包发给B，B下载到Windows

使用与本文一起提供的：

```text
team47_nuc_check_20261016.tar.gz
```

这是本次检查需要的**源码包**，含四个包：gimbal_common、gimbal_interfaces、gimbal_perception、gimbal_tracker_bringup，以及专用视觉入口和只读小工具。源码来自本地仓库提交`e5bd9d0`，额外附带本次检查入口。

Mac上的build/install/虚拟环境不能当NUC程序直接用，所以传源码，在NUC Ubuntu里编译。B不用先在Windows解压，也不必重新从Git挑文件。

### 步骤2：B在NoMachine里为上传文件建一个专用目录

连接NUC后，在远程Ubuntu桌面按`Ctrl+Alt+T`，这是**NUC终端2**。输入：

```bash
mkdir -p ~/ws_championship/team47_transfer_20261016
printf '%s\n' "$HOME/ws_championship/team47_transfer_20261016"
```

应该显示类似：

```text
/home/dut0bug/ws_championship/team47_transfer_20261016
```

用户名以实际输出为准，记住这一整条路径。

### 步骤3：B用NoMachine上传压缩包

1. 打开NoMachine的`!M`菜单。菜单隐藏时可尝试`Ctrl+Alt+0`打开连接菜单。
2. 选择`Transfer a file` → `Upload file from the client`。这里client就是B的Windows。
3. 在Windows文件选择框中选刚下载的`team47_nuc_check_20261016.tar.gz`。
4. 远端保存位置选择步骤2打印的目录。
5. 等传输结束；`Show transfer status`可以查看传输状态。

参考就在这里：[NoMachine官方文件传输说明](https://www.nomachine.com/sharing-files-between-computers)。如果界面文字略有不同，找“上传到远程电脑”这一方向。

传完，B回NUC终端2输入：

```bash
ls -lh ~/ws_championship/team47_transfer_20261016/team47_nuc_check_20261016.tar.gz
```

**应该看到一行文件信息，最后是这个文件名，文件大小不是0。** 若提示`No such file or directory`，说明没放到这个位置：回NoMachine传输记录找保存路径，先不要继续解压。

### 步骤4：B解压到本队专用源码目录

先看压缩包顶层名字：

```bash
tar -tzf ~/ws_championship/team47_transfer_20261016/team47_nuc_check_20261016.tar.gz | sed -n '1,8p'
```

应该都是以`47-vision-check-20261016/`开头的路径。接着整块复制：

```bash
mkdir -p ~/ws_championship/src
if [ -e ~/ws_championship/src/47-vision-check-20261016 ]; then
    printf '本队目录已存在，先别重复解压或覆盖，核对之前是否已经部署。\n'
else
    tar -xzf ~/ws_championship/team47_transfer_20261016/team47_nuc_check_20261016.tar.gz -C ~/ws_championship/src
    printf '解压命令已结束，下面检查文件。\n'
fi
ls ~/ws_championship/src/47-vision-check-20261016/src
```

正常应该看到四个目录：

```text
47-gimbal_common  47-gimbal_interfaces  47-gimbal_perception  47-gimbal_tracker_bringup
```

若解压报错，保存终端内容，不接着编译。已有目录时也不删除旧文件；先核对再处理。

### 本次到底会往NUC写哪些东西？

| 位置 | 放什么 |
|---|---|
| `~/ws_championship/team47_transfer_20261016/` | 上传的压缩包 |
| `~/ws_championship/src/47-vision-check-20261016/` | 本次四个包和检查工具 |
| `~/ws_championship/team47_check_20261016/` | 本队编译结果、安装结果、日志和采集数据 |

不覆盖原`~/ws_championship/install`，不覆盖已有相机、雷达、云台驱动。包内已放`COLCON_IGNORE`标记：别人按常规扫描整个工作区时，会跳过这份检查副本；本队命令直接从内部src开始，只发现这四包。**保留这个标记，不要删除，也不要把它移进内部src。** 本队构建/记录目录也会创建同样标记。依据是选手工作区要求及ROS的独立安装前缀机制：[比赛环境说明](https://blog.secaumene.com/kit/campus-contest-env.html)、[ROS2工作区教程](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html)。

忽略标记的技术依据：[colcon官方目录扫描实现](https://raw.githubusercontent.com/colcon/colcon-recursive-crawl/master/colcon_recursive_crawl/package_discovery/recursive_crawl.py)。照本文的指定路径命令执行即可。

## 四、B在NUC编译，确认运行的是我们的版本

### 步骤5：在终端2加载本队检查环境

```bash
source ~/ws_championship/src/47-vision-check-20261016/进入检查环境.bash
```

应该显示：

```text
队伍源码：/home/实际用户名/ws_championship/src/47-vision-check-20261016
队伍构建与记录：/home/实际用户名/ws_championship/team47_check_20261016
ROS域：0（保留原设置）
本终端只使用系统Humble和本队专用安装目录。
```

ROS域也可能是其它数字，显示什么就记录什么。这是ROS通信分组，实机保持与现有设备相同；今天不改成87。该文件只整理**当前终端**的环境，不会改其它正在运行的进程。

后续命令里的`$T47_SRC`和`$T47_RUN`就是上面两条路径的简写，不需要手动输入用户名。

### 步骤6：检查现有依赖，今天不现场升级

```bash
python3 -c 'import rclpy,numpy,yaml,cv2; print("OpenCV",cv2.__version__); print("aruco",hasattr(cv2,"aruco"))'
colcon --log-base "$T47_RUN/log" list --base-paths "$T47_SRC/src"
```

应该：第一条显示OpenCV版本和`aruco True`；第二条列出前面四个ROS包各一次。目录名前有47不影响ROS包名。

出现`ModuleNotFoundError`、`aruco False`或`colcon: command not found`：保存报错，今天先做第六、七、九部分并采原图；不在公用NUC执行pip/apt安装脚本。

### 步骤7：只编译这四个包

整块复制执行：

```bash
cd "$T47_SRC"
CMAKE_BUILD_PARALLEL_LEVEL=2 colcon --log-base "$T47_RUN/log" build \
  --base-paths "$T47_SRC/src" \
  --packages-select gimbal_common gimbal_interfaces gimbal_perception gimbal_tracker_bringup \
  --build-base "$T47_RUN/build" \
  --install-base "$T47_RUN/install" \
  --executor sequential \
  --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release
```

正常过程会有`Starting >>> ...`、`Finished <<< ...`，最后类似：

```text
Summary: 4 packages finished [...]
```

一些setuptools警告不等于失败；**看到Failed或Aborted就停止后面的视觉启动**。保留第一条错误和最后Summary，详细日志在`$T47_RUN/log`。若需要恢复，先只处理这一处错误，不扩大成全工程编译。

注意：这批代码已经做过离线检查，但这里四包的真实Humble编译仍需你们现场确认。教程不会把尚未做过的NUC构建写成已成功。

### 步骤8：让终端使用刚编译的版本

```bash
source "$T47_SRC/进入检查环境.bash"
ros2 pkg prefix gimbal_perception
ros2 interface show gimbal_interfaces/msg/MarkerCorners
```

第一条查询应该显示：

```text
/home/实际用户名/ws_championship/team47_check_20261016/install/gimbal_perception
```

**必须是team47_check_20261016下面的路径。** 若是原`~/ws_championship/install/...`，说明仍在用旧环境，先重新执行本步骤，不启动视觉。

第二条应包含`marker_id`、`corners`、`valid`等字段。若说找不到MarkerCorners，保留构建Summary与包路径，停止视觉启动；这是新消息未正确安装或环境未加载。

## 五、三个终端都怎样准备？

B在NUC再开终端1、终端3。在**每个新终端**先输入：

```bash
source ~/ws_championship/src/47-vision-check-20261016/进入检查环境.bash
```

三者显示的ROS域必须一致。终端1稍后保持视觉运行，终端2检查/改参数，终端3录bag。终端关闭后环境变量就没有了，新终端重新source即可。

本次视觉的业务输出都放在`/team47_check/`下面，避免给已有`/perception/...`串数据。已有其它节点的日志/参数系统话题仍可能正常出现。为什么单独名字还要设置话题映射，参考：[ROS2命名空间和重映射](https://docs.ros.org/en/humble/How-To-Guides/Launch-file-different-formats.html)。

## 六、B检查相机：输入什么，正常是什么

### 步骤9：终端2确认相机在发数据

```bash
ros2 topic info /image_raw --verbose
```

应该看到类似：

```text
Type: sensor_msgs/msg/Image
Publisher count: 1
...
Node name: hik_camera
...
Reliability: RELIABLE
```

节点名以现场为准。重点看三项：Type是Image；Publisher count至少1；Reliability记录下来。多个publisher时保存整段输出，先不把它当唯一相机源。

接着：

```bash
ros2 topic echo /image_raw --once --field width --qos-reliability best_effort
ros2 topic echo /image_raw --once --field height --qos-reliability best_effort
ros2 topic echo /image_raw --once --field encoding --qos-reliability best_effort
ros2 topic echo /image_raw --once --field header --qos-reliability best_effort
```

前三条预期分别为`1440`、`1080`、常见的`rgb8`或`bgr8`；也可能是支持的mono8。第四条有`stamp`和`frame_id`；再执行一次，时间应变大。

best_effort是这几条读数命令的接收设置，不会改驱动。当前检测程序按官方大图像流要求使用Reliable订阅；若发布者实际是BEST_EFFORT，先保留输出交回处理，不能照原入口声称已兼容。

**异常就这样做：**

| 看到什么 | 当场做什么 |
|---|---|
| `Unknown topic`或Publisher count为0 | 输入`ros2 topic list -t`，保存结果；检查是否在NUC终端、ROS域是否与驱动一致 |
| echo等5秒还无输出 | Ctrl+C结束这条命令，保存topic info，不重开驱动 |
| 尺寸不是1440×1080 | 记录实际尺寸；今天不按现有K验收测距，先查驱动缩放/裁剪 |
| `/camera_info`的K全零 | 本次视觉用已标定的队伍YAML，不用这份全零K，也不重新标定 |

在已有RViz里用`Add → By topic → /image_raw → Image`看画面。画面清晰、码完整进入画面后再做视觉。

为什么尺寸和K必须对应：[第二课，相机模型与畸变](https://blog.secaumene.com/teach/class-2.html)。K就是把光线方向映射到像素位置的参数；图被缩放/裁剪后，旧K未必还能直接用。

## 七、B检查雷达和IMU：逐条读懂输出

**点云是雷达扫到的一堆三维点；IMU是加速度/角速度传感器。Livox雷达内有一个IMU，云台还有自己的IMU，它们是两个数据源。**

参考：[主办方MID360说明](https://blog.secaumene.com/kit/mid360-lidar.html)。本节重点是“有无连续消息、类型是什么”，不是直接做定位。

### 步骤10：终端2查询雷达点云的类型

```bash
ros2 topic info /livox/lidar --verbose
```

正常可能是以下两种之一：

```text
Type: livox_ros_driver2/msg/CustomMsg
Publisher count: 1
```

或者：

```text
Type: sensor_msgs/msg/PointCloud2
Publisher count: 1
```

**怎么判断：**

- Publisher count为1：有一个程序声明在发这个话题，还要继续看消息是否流动。
- CustomMsg：Livox专用格式，是本项目现有Point-LIO的候选输入。
- PointCloud2：ROS通用点云格式，可以用来显示；不能直接交给当前按CustomMsg配置的Point-LIO。

这两种格式不是“哪个高级”，而是接收程序要求不同。本次不改雷达驱动的格式。

如果`/livox/lidar`不存在，输入：

```bash
ros2 topic info /red_standard_robot1/livox/lidar --verbose
```

若第二个有publisher，后续点云命令用这个全名。两个都有时，这小时先选一个有真实消息的记录，不花时间猜是否同一设备；把两份info留给后续对照。

### 步骤11：看点云是否真的更新

如果步骤10显示的是PointCloud2，终端2输入：

```bash
ros2 topic echo /livox/lidar --once --field header --qos-reliability best_effort
```

如果实际选了前缀全名，在命令里一起换掉。预期：

```text
stamp:
  sec: 179...
  nanosec: ...
frame_id: livox_frame
```

frame_id可能不同，以现场为准。再执行一次，时间戳应增加。只读取header，避免把整片点云打印满屏幕。

**若类型是CustomMsg，也可以尝试同一条header命令，但本队最小环境未必装有Livox自定义消息。** 若报`The message type ... is invalid`，它表示这个终端缺消息定义，不能据此判断雷达坏了。此时先看已有RViz：若原本已显示雷达点云，记录其Topic与Fixed Frame，拍下当前点云。若没有现成显示，保存info和错误，今天继续检查标准IMU，不额外启动驱动或读取保护目录。

有publisher但读不到消息/时间不变，也先保存结果；“话题名字存在”不算数据通过。

### 步骤12：读取雷达IMU，由小工具算出模长

先输入：

```bash
ros2 topic info /livox/imu --verbose
```

预期：`Type: sensor_msgs/msg/Imu`、Publisher count至少1。短名不存在就查`/red_standard_robot1/livox/imu`，后续命令使用实际全名。

C让雷达保持静止，B终端2输入：

```bash
python3 "$T47_SRC/tools/read_imu.py" /livox/imu
```

最多约8秒后自动结束，不需要人工算平方根。示例：

```text
已收到 30 条不同时间戳消息；frame_id=livox_frame
最后加速度xyz：0.01200, -0.01800, 1.00200
加速度模长中位数：1.00223；角速度模长中位数：0.00120
加速度数值约1：与g单位相符，仍须核对驱动定义。
姿态：未提供有效四元数；雷达IMU可能正常只提供加速度/角速度。
```

这些数是读法示例，**不是要求一模一样**。只看：连续不同时间戳有没有收到；静止时角速度是否小而稳定；加速度模长更接近1还是9.8。倾斜时三个分量会重新分配，不要求只有z有值。

| 实际结果 | 判断和下一步 |
|---|---|
| 连续消息，加速度模长约1 | 与g单位相符；记录，后续对照驱动定义。当前LIO配置acc_norm=1.0只适用于这一单位 |
| 模长约9.8 | 与m/s²相符；记录，不能沿用按g设定的LIO参数 |
| 四元数未提供，但加速度/角速度连续 | 对雷达IMU可能正常，不因此判驱动坏 |
| 读不到连续消息 | 保存topic info；核对全名、ROS域，今天不启动定位 |
| 静止却数值大跳、非有限值或时间异常 | 保存原始样本，先不验收LIO |

单位为什么影响定位：[MID360教程的IMU/Point-LIO相关内容](https://blog.secaumene.com/kit/mid360-lidar.html)。小工具只做初筛，单位以对应驱动的实际定义确认。

### 步骤13：读取云台IMU，今天不碰串口

```bash
ros2 topic info /red_standard_robot1/gimbal --verbose
```

若Type是`sensor_msgs/msg/Imu`且有publisher，输入：

```bash
python3 "$T47_SRC/tools/read_imu.py" /red_standard_robot1/gimbal
```

静止时通常应有：连续时间戳、小而稳定的角速度、约9.8m/s²的加速度模长；四元数模长接近1。若云台IMU提示未提供有效四元数，记录为姿态链待排查，不能直接送进后续世界坐标换算。

如果没有话题，执行`ros2 topic list -t`，查是否是别的云台IMU名字。仍没有就记“设备已上电，未找到ROS反馈”，先完成视觉。串口和桥接问题另行处理，不影响本次相机坐标内的测距。

四元数、加速度和坐标的定义在这里：[统一协议](https://blog.secaumene.com/kit/protocol.html)。现在只检查数据，不把惯性yaw直接当机械关节角。

## 八、B运行本队视觉，A/C摆码验收

### 步骤14：先知道手里这张码的ID

把一张平整ArUco码放进相机画面。B终端2输入：

```bash
python3 "$T47_SRC/tools/read_marker_ids.py"
```

工具只读取相机，约10秒自动结束。正常类似：

```text
识别到 ArUco ID：3
检查结束：处理图像...帧，发现ID=[3]
```

记下真实ID。出现多个ID时，C先移走其它码，只留当前一张。若“处理图像0帧”，回相机/QoS检查；若有图但ID=[]，检查码完整、清晰、无强反光，字典是否匹配DICT_4X4_50。

ArUco的ID怎样编码、为什么不是数字OCR：[OpenCV官方ArUco检测教程](https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html)。

C量**黑色标记外边界**的边长，不含外围白纸，比赛码应为100mm。这个长度直接影响测距；若手里码不是100mm，今天可验识别/符号，先不按默认0.10m验收绝对距离。

### 步骤15：B在终端1启动本队专用视觉

若上一步识别到3，输入：

```bash
ros2 launch gimbal_tracker_bringup team47_vision_check.launch.py target_id:=3
```

识别到其它ID就把3换成实际值。**保持终端1运行。** 正常应看到两个进程启动，PnP日志包含：

```text
内参源=yaml（.../camera_team47.yaml），fx=1778.23，marker=0.1m
```

若日志用的不是camera_team47.yaml或fx明显不对，Ctrl+C停止本次launch，保存日志。不要继续拿结果验收。

终端2核对：

```bash
ros2 node list
ros2 param get /team47_check/aruco_detector_node perception.target_digit
ros2 param get /team47_check/marker_pnp_node perception.target_digit
ros2 topic echo /team47_check/marker_cam --once --qos-reliability best_effort
```

应有本队两个节点，两个参数同为所选ID；码可见时的消息类似：

```text
header:
  stamp: ...
  frame_id: camera_optical_frame
marker_id: 3
x: 0.02
y: -0.01
z: 1.50
...
reproj_err_px: 0.4
valid: true
```

**看哪几个值：** marker_id是当前码；valid为true才是有效测量；x/y/z单位米；z>0；当前默认重投影误差门限2像素。示例x/y/z不是你们一定会得到的值。

valid=false表示这一帧没有可信的位置，可能是没看见指定码、图像过旧或PnP拒绝。要查原因，不能拿无效帧的0坐标当真实目标。

PnP为什么能从已知边长的四个角点算位置：[第二课，相机模型/PnP](https://blog.secaumene.com/teach/class-2.html)。今天不需要先推导全部公式，但应能解释输入是角点、K/D和码尺寸，输出是码在光学坐标里的位置。

### 步骤16：三种坐标的方向，现场怎么认？

先固定视角：**A站在相机后面，沿镜头拍摄方向看。** 下面的左右都按这个视角。

| 名字 | 原点 | +x | +y | +z | 如何确定 |
|---|---|---|---|---|---|
| O：光学坐标camera_optical_frame | 相机光心 | 图像右 | 图像下 | 镜头看出去 | 按相机模型定义，再用下面移码实验核验 |
| C：相机坐标camera_link | 与O同一光心 | 镜头看出去 | 图像左 | 图像上 | ROS相机安装坐标约定；与O是同一实物的不同轴命名 |
| G：云台IMU输出坐标gimbal_link | IMU参考点 | 固件定义的前 | 固件定义的左 | 固件定义的上 | 对应实际固件映射和安装图，不能凭芯片丝印或镜头方向猜 |

C的“上”是图像上，镜头抬头/滚转时不一定等于桌面竖直。G与C是否同向，是要确认的安装事实，不能因相机在云台上就假定同向。

已知的C/O关系用一句话记：**O坐标(x,y,z)换成C坐标就是(z,-x,-y)**。例如码在图像右侧，O的x为正，C的y为负，都是同一个“偏右”，不矛盾。

参考：[第一课，坐标变换和TF](https://blog.secaumene.com/teach/class-1.html)、[ROS官方坐标约定REP-103](https://www.ros.org/reps/rep-0103.html)。包内教学说明也写了这些约定：`gimbal-edu-tools/gimbal_edu_description/README.md`；其中默认外参数值只是初值，不是你们的实测结果。

### 步骤17：C移动目标，A看方向，B看连续数值

终端2输入（这次不要--once）：

```bash
ros2 topic echo /team47_check/marker_cam --qos-reliability best_effort
```

C每个动作保持约5秒，B看valid=true的消息：

| C做什么 | B应该看到 | A记录什么 |
|---|---|---|
| 同一深度向图像右移动 | x增大 | 右移：通过/失败 |
| 同一深度向图像下移动 | y增大 | 下移：通过/失败 |
| 沿光轴移远 | z增大 | 移远：通过/失败 |
| 完全遮挡2秒，再拿开遮挡 | 遮挡无新有效观测；恢复valid=true且stamp更新 | 遮挡/恢复现象 |

结束Ctrl+C，只停止终端2这条echo，终端1视觉保持运行。

这个实验能核验O轴和PnP方向，**不能单凭它确定G轴**。G的前/左/上要找固件输出映射所对应的物理安装基准；只有静止加速度，最多约束重力方向，不能单独找出绝对“前”。如果G轴还不明确，就拍安装图和型号带回，今天仍可完成O系测距。

### 步骤18：近、中、远三个位置，分别量、分别录

空间够就取约1m、2m、3m；空间不够用三个能准确量的位置。C让码平整、尽量正对镜头，码中心靠近图像主点约(716,589)。

A先按下一节的方法选光心代理基准。每档写实际尺量距离、量的是哪两点、光心位置是否未知；不只写“放了1米”。

B终端3输入：

```bash
ros2 bag record -o "$T47_RUN/records/pnp_near" /team47_check/marker_corners /team47_check/marker_cam
```

C保持近档静止，录5—8秒后B在终端3按Ctrl+C。再摆中档，把目录名换成pnp_mid；远档换pnp_far，分别录。目录已存在就换near_02等新名，不覆盖。

现场看z是否接近独立尺量的前向深度。平均值/波动可以回来用bag统计。偏轴时尺子拉的是斜距，应与`sqrt(x²+y²+z²)`比，不能直接对z。

独立基准足够准确时，先用`max(3cm,距离×3%)`筛明显问题，例如2m对应6cm。这个门槛是团队排错参考，非官方精度标准。光心代理偏移未知时，只做粗检，不宣称厘米级外参已标定。

### 视觉遇到问题，按这里处理

| 看到什么 | 下一步 |
|---|---|
| launch文件找不到 | 核对步骤8包路径；本次用team47_vision_check.launch.py，不是旧competition入口 |
| 没有输出话题 | 看终端1是否有进程退出/异常，保存最先出现的错误 |
| 工具能识别ID，视觉却一直invalid | 核对两个参数ID、PnP日志K/尺寸、角点话题；保存拒绝原因 |
| 有图但码识别不到 | 靠近、摆平、去反光，确保四边完整；不先调滤波器 |
| 距离整体成固定倍数偏差 | 查黑色外边长、单位和图像分辨率/K是否一致 |
| 遮挡后仍持续报告新有效观测 | 留下连续消息/短bag，作为具体缺陷带回，不把它当通过 |

若视觉构建/运行失败，至少录原始数据：

```bash
ros2 bag record -o "$T47_RUN/records/raw_camera_near" /image_raw /camera_info
```

A/C按实际测距位置摆码，只录3—5秒Ctrl+C；原图体积大，不连续录几分钟。保留原图bag和对应距离，回来才能复现检测。

## 九、A/C量安装关系：把“投影到三个方向”换成实际尺量步骤

### 步骤19：先确定相机原点和IMU原点，确定不了就用代理点

**先分清实物：镜头/相机外壳属于相机；云台两只大电机的中心、转轴交点都不能直接当IMU原点。IMU通常是安装板上的传感器或模块，可能藏在外壳里，现有照片不足以可靠指出其参考点。**

**相机光心**是投影模型使用的参考点，一般藏在镜头内部，肉眼不能从镜头前玻璃直接认出来。棋盘格内参K/D也不给它相对外壳的位置。优先按相机/镜头机械资料中的投影中心或可信标定结果确定；没有就先选镜头上可重复定位的外部点P，例如某一镜头安装肩面中心，并拍照写清。

**IMU原点**按设备/固件文档所定义的参考位置取。看到芯片不等于确定了定义：输出坐标也可能经过固件旋转映射。先找IMU型号、板子位置、安装图和固件说明；原点看不到时选可重复找到的外部点Q，例如IMU安装板上某个明确螺孔中心，并注明Q到IMU原点的偏移未知。

**先量P相对Q。** 两个真实参考点的位置偏移未确认前，这叫“机械基准初测”，不把它写成光心相对IMU的精确外参。拍下P/Q位置、所用边面和型号标签，回来可以补查机械资料。

对应原理：[第二课，相机模型/PnP](https://blog.secaumene.com/teach/class-2.html)、[第一课，坐标原点与变换](https://blog.secaumene.com/teach/class-1.html)。找不到光心不会阻塞今天的识别和方向检查。

### 步骤20：先在纸上画出两点，分开量前后、左右、高低

下面先假设G的前/左/上已确认，而且该姿态G上方向竖直。若不满足，直接跳到步骤21记录原始尺寸，不硬套。

需要的是三项差值。把它想成两人在地图上位置不同：前后差、左右差、高度差分别量，斜线总长不够用。

**量前后差：**

1. C在安装件上选一条与G前方向垂直的、可同时对照P/Q的共同基准面。
2. 用直角尺使读尺方向平行G前方向，分别读“基准面到Q”和“基准面到P”的距离。
3. A做“P读数减Q读数”；基准面在后、读数向前增加时，正值就是P更靠前。
4. 例如Q读30mm、P读110mm，前后差为+80mm。若共同基准在前、读数向后增大，符号要反过来；把所用方向画在照片上。

**量左右差：**

1. 俯视两点，找平行G前方向的共同直边；若没有，直角尺建立一条临时参考线。
2. 分别读P/Q到它的侧向距离，计算P相对Q偏左还是偏右。
3. P偏左20mm记+20mm，偏右20mm记-20mm。不能把“离尺更远”一律当正，正方向由G左方向决定。

**量高低差：**

1. 用同一底面/桌面作高度基准，竖直量Q高度hQ、P高度hP。
2. 算hP-hQ。P更高50mm，记+50mm；P更低记负。
3. 这一步只有在G上方向确实竖直时，才能直接作为G的z分量。

每项重复三次。示例表只是帮助理解，现场用你们的读数：

| 项目 | Q读数 | P读数 | P相对Q的差 |
|---|---:|---:|---:|
| 前后，共同基准在后，朝前增大 | 30mm | 110mm | +80mm |
| 左右，P比Q偏右 | — | — | -20mm |
| 高度，同一底面 | 100mm | 150mm | +50mm |

于是P相对Q为(80,-20,50)mm，换米为(0.080,-0.020,0.050)。**这是算式例子，不是本机参数；若P/Q是代理点，还没有得到真实光心—IMU平移。**

为什么必须用同一坐标轴表达平移：[第一课，刚体变换](https://blog.secaumene.com/teach/class-1.html)。今天先把两点和三个分量量清楚，矩阵计算回来做。

### 步骤21：G轴倾斜、原点不明或没公共基准时，现场怎样继续？

- **G轴倾斜：**在纸上定义一个临时测量坐标H：沿桌面上一条明确直边向前为Hx、桌面左为Hy、桌面竖直向上为Hz。按H量P/Q差，记“这是H系，不是G系”；拍下桌边和G安装的相对姿态。没有H与G旋转依据时，先不换算。
- **G方向不明：**记录型号、IMU位置、板子安装朝向，G三轴填写“未确认”。光学O/C方向仍按步骤16/17核验，视觉实验照做。
- **没法直接伸尺：**量两个点各自相对同一个可接近基准的位置，再相减。若连共同基准也不可定位，只拍照和记录缺项，不凭肉眼写毫米数。
- **相机/IMU没有固定在同一随pitch转动的部件上：**拍清谁固定在底座、谁随俯仰臂运动，画运动链；两者关系会随关节改变，不能填一组常量结束。

本次测量交付：三视图标P/Q和可确认的轴；每项三次原始尺寸；单位；测量时云台姿态；代理点到真实原点的未知偏移。**尚未装到底盘的雷达和云台，不测“最终底盘外参”。**

### 旋转今天怎么记录？

A/C同时看两件事：镜头看出去的方向，图像上方向。两者都和G对应轴有可靠平行安装依据，才能记录“近似同向的机械初值”。只有镜头大概朝前还不够，因为相机可能绕光轴滚转。

有安装图/角度基准就拍下并量；没有就留未知。**不靠试正负把程序调得像正确，也不把未知旋转填成0。** 后续可利用独立测量的多个目标位置标定或核验安装变换；这小时先把可重复的物理基准留下来。

## 十、还有余量时，只读已有Point-LIO输出

这轮先完成视觉与真实接口，不在公用NUC上临时拉起完整导航、Nav2或新雷达驱动。已有定位节点时可以检查：

```bash
ros2 topic info /red_standard_robot1/aft_mapped_to_init --verbose
ros2 topic echo /red_standard_robot1/aft_mapped_to_init --once
```

正常类型是nav_msgs/msg/Odometry，消息有header、child_frame_id、pose。现有工程通常parent=camera_init、child=body；现场以真实消息为准。若没有这个话题，今天记录“未找到此定位输出”，不把它等同源码缺失。

固定雷达不动，隔约30秒再读一次，记录位置有没有明显跳变。**这测的是该输出参考点的静止表现；雷达没与底盘固定、未换算杠杆臂时，不能称为底盘中心定位。** twist全零也可能只是程序没有填速度。

Point-LIO是什么、为什么还需要坐标变换：[导航实操](https://blog.secaumene.com/nav-intro/practice.html)、[定位、规划与控制](https://blog.secaumene.com/nav-intro/localization-planning-control.html)。真正启动新LIO放在点云类型、IMU单位和固定安装确认后单独安排。

## 十一、离场前：把本队资料真正下载回来

### 步骤22：B先确认录包里真的有消息

终端3所有录包都Ctrl+C结束后，在终端2输入：

```bash
ros2 bag info "$T47_RUN/records/pnp_near"
```

应有marker_corners/marker_cam两个话题、Message Count大于0。若今天录的是原图，换成raw_camera_near。三个距离对应的包都核对一次。只复制metadata.yaml不够，数据库文件也要一起带走。

A把下面记录写成一个文本文件，放本队records里；C把照片保留在手机并按近/中/远编号，随后发给A整理。

```text
日期与操作人：
NUC主机名/ROS域：
本队包prefix路径/构建Summary：
相机尺寸、编码、frame、QoS：
码ID、黑色外边长：
右移/下移/移远/遮挡结果：
近、中、远实际尺量值及量取基准：
光心或IMU原点未确定的偏移：
P/Q照片、轴依据、三次尺寸：
雷达点云全名和类型：
雷达IMU读数/单位依据：
云台IMU全名和读数：
零位/限位已知资料、未确定项：
首次错误和未完成项目：
```

### 步骤23：B停止本次视觉，再打包自己的记录

终端1按Ctrl+C停止本队视觉；不关闭原相机/雷达/云台驱动。

终端2输入：

```bash
REPORT_ID=$(date +%Y%m%d_%H%M%S)
tar -czf "$T47_RUN/team47_records_$REPORT_ID.tar.gz" -C "$T47_RUN" records
ls -lh "$T47_RUN"/team47_records_*.tar.gz
```

应该看到新生成的结果包和非零大小。用NoMachine `Transfer a file → Download file from the server`，从NUC选这个结果包下载到B的Windows。核对Windows里确实有该文件，再离场。[NoMachine文件传输参考](https://www.nomachine.com/sharing-files-between-computers)。

本次代码、安装和日志都留在前面的本队专用目录；不在离场时临时清理其它目录。原图bag和结果包单独保存，不作为源码直接推GitHub。

## 最后按这五项判定今天做成了什么

1. **部署通过：**只编译本队四包；gimbal_perception的prefix是专用安装路径。
2. **输入通过：**原图尺寸/时间可读；雷达和IMU的实际全名、类型与连续消息情况有记录。
3. **视觉通过：**正确ID、valid=true；右/下/远符号正确；遮挡/恢复符合预期。
4. **数据可复现：**三个距离或至少一段原图有bag，且有独立尺量基准；安装照片/尺寸对应得上。
5. **资料带走：**结果包已下载到Windows，未完成项明确写原因。

这些通过以后，下一步才是用现场数据核验安装外参和时间链、接入定位，再推进云台闭环。每项以实际结果为准；单独视觉通过不代表整车比赛系统已经通过。

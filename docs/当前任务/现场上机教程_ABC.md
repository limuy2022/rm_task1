# 现场操作教程：从传文件开始，一步一步做

更新：2026-10-17。**本文件供后续按需参考；本次只做[G核查](G坐标定义核查_实际操作.md)，不从第1步重新做整套流程。**

**B在Windows打开NoMachine；下面所有命令都输入NoMachine远程桌面里的NUC Ubuntu终端。** Windows只负责选文件上传和下载，不在PowerShell执行这些Linux命令。

相机、雷达、云台已经启动，直接用现有数据。**用户补充：昨天第4步首次全部验证通过，message type invalid出现在后续重试；O/C坐标与转换已明确。** 本轮直接按第7步的[G坐标核查](G坐标定义核查_实际操作.md)推进，不重做视觉与O/C。其余步骤保留为按需参考。

## 先说明今天拿哪个文件

只需要发给B这一个包：

```text
team47_onsite_20261017.tar.gz
```

它把查看结果、读IMU、相机标定需要的工具放在一起；也带了四个视觉包的源码备用。**昨晚的安装还在就直接复用，不重新编译。** 如果安装不在，按本文附录A重新部署。

本轮复用昨晚已运行的本队视觉安装；10月17日代码补丁不会因传入这个包而自动应用。相机标定、IMU读取和路径诊断使用本包内的工具。

Mac上的build/install不适合直接搬到NUC运行。这里传的是源码和Python工具，需要编译的部分由NUC自己编译。下面会明确指出哪一步需要编译。

A带尺、直角尺、笔；C带平整棋盘格和ArUco码。三人共用这份操作顺序，不各改一份配置。

**出发前：**A从本对话下载上面的tar.gz文件，作为文件发给B；B把它保存到Windows的“下载”文件夹。下面第1步从这个Windows文件开始上传。

## 第1步：B把文件从Windows传到NUC

**B做：**在NoMachine远程Ubuntu桌面按Ctrl+Alt+T，打开终端1，输入：

```bash
mkdir -p ~/ws_championship/team47_transfer_20261017
printf '%s\n' "$HOME/ws_championship/team47_transfer_20261017"
```

**应该显示：**

```text
/home/dut0bug/ws_championship/team47_transfer_20261017
```

用户名以实际输出为准。记下这条完整路径。

**下一步：**打开NoMachine的`!M`菜单 → `Transfer a file` → `Upload file from the client`。在Windows里选`team47_onsite_20261017.tar.gz`，远端保存位置选刚才打印的目录。传完回NUC终端1输入：

```bash
ls -lh ~/ws_championship/team47_transfer_20261017/team47_onsite_20261017.tar.gz
```

**应该显示：**一行文件信息，最后是这个文件名，文件大小不是0。

**如果不同：**`No such file or directory`表示没有放到这个路径。先在NoMachine传输记录里找实际保存位置，别接着解压。[NoMachine文件传输说明](https://www.nomachine.com/sharing-files-between-computers)放在这里，方便对照菜单。

## 第2步：B解压工具，三个终端都加载同一个环境

**终端1输入：**

```bash
if [ -d ~/ws_championship/team47_transfer_20261017/team47_onsite_20261017 ]; then
    printf '本次工具目录已存在，跳过解压。\n'
else
    tar -xzf ~/ws_championship/team47_transfer_20261017/team47_onsite_20261017.tar.gz -C ~/ws_championship/team47_transfer_20261017
fi
source ~/ws_championship/team47_transfer_20261017/team47_onsite_20261017/进入本次操作.bash
```

**应该显示：**本队源码路径、本队构建路径、本次工具路径、本次记录路径。例如：

```text
队伍构建与记录：/home/dut0bug/ws_championship/team47_check_20261016
本次记录：/home/dut0bug/ws_championship/team47_check_20261016/records/onsite_20261017
```

看到这些就表示这个终端知道该去哪里找本队程序和记录。`$KIT`是工具目录的简写，`$LAB`是本次记录目录的简写，不用手动替换用户名。

**如果显示“没有找到昨晚的本队源码目录”：**先做本文附录A，再回来继续。不在整个工作区运行colcon build。

**B再开终端2和终端3，每个都先输入这一行：**

```bash
source ~/ws_championship/team47_transfer_20261017/team47_onsite_20261017/进入本次操作.bash
```

以后分工固定：终端1运行视觉；终端2看结果和查设备；终端3录数据或运行标定。**每个新终端都要source；终端1的环境不会自动传给终端2。** [ROS工作区与source说明](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html)。

### 这些文件会不会弄乱公共NUC？

只用三个本队位置：

| 位置 | 放什么 |
|---|---|
| `~/ws_championship/team47_transfer_20261017/` | 上传的工具包 |
| `~/ws_championship/src/47-vision-check-20261016/` | 本队四个视觉包的源码 |
| `~/ws_championship/team47_check_20261016/` | 本队编译、安装、日志和采集数据 |

原相机、雷达、云台驱动不替换。源码父目录和本队构建目录有COLCON_IGNORE，别人正常扫描全工作区时会跳过；本队编译只从内部src开始。本次日志也放进本队records。**本轮不使用原competition.launch.py。**

## 第3步：仅当重试再次出现“message type invalid”时使用

首次视觉已通过；本步用于后续异常的环境排查，不作为本轮G任务的前置重测。

**终端2输入：**

```bash
ros2 pkg prefix gimbal_interfaces 2>&1 | tee "$LAB/interfaces_prefix.txt"
ros2 pkg prefix gimbal_perception 2>&1 | tee "$LAB/perception_prefix.txt"
```

**应该显示：**两个路径都在：

```text
/home/实际用户名/ws_championship/team47_check_20261016/install/...
```

**如果指向别处：**重新执行第2步的source，再查一次。仍不对就把两条输出发来，不继续混用旧包。若显示找不到包但源码还在，说明安装可能被清理；按附录A复用源码重新编译，别再复制覆盖。

**然后终端2输入：**

```bash
python3 -c 'from rosidl_runtime_py.utilities import get_message; from rclpy.type_support import check_for_type_support; c=get_message("gimbal_interfaces/msg/MarkerObservation"); check_for_type_support(c); print("消息类型加载成功")'
```

**应该显示：**`消息类型加载成功`。

**如果仍报错：**原样保留整段错误。再输入下面这条，把诊断保存下来：

```bash
python3 "$KIT/tools/inspect_nuc.py" --expected-install "$T47_RUN/install" --output "$LAB/message_and_serial_check.json"
```

把生成的JSON带回。这个检查只查安装环境和设备路径，不打开串口。不要为这条错误重装ROS；A/C仍可做第7、8步的实物记录。

## 第4步：本队视觉已首次验证通过，以下保留启动参考

### 4.1 先看相机原始画面，再确认码的ID

**终端3输入：**

```bash
cd "$LAB"
ros2 run image_view image_view --ros-args -r __node:=team47_image_view -r __ns:=/team47_check -r image:=/image_raw
```

**应该出现：**相机实时画面窗口。C拿一张纸在镜头前移动，窗口也应同步变化；这是在看已有/image_raw，没有重新启动相机驱动。本队视觉节点本身只输出数据，不会自动开这个窗口。[image_view官方实现与参数](https://github.com/ros-perception/image_pipeline/blob/humble/image_view/src/image_view_node.cpp)。

**如果不同：**Package not found就记“预览包不可用”，不现场安装；第9步的标定窗口仍可看图，第4.3步需要对照画面的方向验证先记未做。若窗口黑着不更新，在终端2查`ros2 topic info /image_raw --verbose`并保存输出；不要另启动一个相机驱动。

窗口正常时先保持终端3运行，C对照它把码完整放进画面。

你们昨晚用了ID=3。如果今天拿的还是同一张，继续用3。不确定时，**终端2输入：**

```bash
python3 "$KIT/tools/read_marker_ids.py" 2>&1 | tee "$LAB/marker_ids.txt"
```

C把一张码完整放在画面里。约10秒后，应该显示`识别到 ArUco ID：3`之类的结果。这个数字才是ID，不是队号47。[ArUco检测原理](https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html)。

### 4.2 终端1启动，保持运行

```bash
ros2 launch gimbal_tracker_bringup team47_vision_check.launch.py target_id:=3 2>&1 | tee -a "$LAB/vision_console.txt"
```

如果刚识别到别的ID，把3换成那个数。

**应该看到：**两个进程启动；PnP日志里的文件为camera_team47.yaml、fx约1778.23。这是当前原配置，不代表它已经通过测距。

**下一步：终端1保持运行，别按Ctrl+C。终端2输入：**

```bash
ros2 topic echo /team47_check/marker_cam --once --qos-reliability best_effort 2>&1 | tee "$LAB/marker_cam_once.txt"
```

**应该看到类似：**

```text
frame_id: camera_optical_frame
marker_id: 3
x: 0.02
y: -0.01
z: 1.50
reproj_err_px: 0.4
valid: true
```

只认`valid: true`的测量。x/y/z单位是米；z=1.50表示目标相对光心的前向深度约1.5m。这些是输出读法示例，不要求你们的数完全一样。

**如果不同：**

- `message type invalid`：回第3步；不是调相机参数能解决的问题。
- 5秒没有消息：Ctrl+C结束这条echo，确认终端1还在运行、工具环境已source。
- 有消息但valid=false：先把指定码完整、平整地放进画面，再看终端1是否报告PnP拒绝；不要拿无效0坐标当真实位置。
- 终端1出现reproj_2.xxpx：这一帧没通过当前2px门槛，保留日志；今天要核验标定，不直接把门槛放宽。

### 4.3 已完成的O系方向验证：本轮跳过

以下是已执行过的参考步骤，不再安排重复操作。

终端2输入：

```bash
ros2 topic echo /team47_check/marker_cam --qos-reliability best_effort 2>&1 | tee "$LAB/marker_cam_moves.txt"
```

| C怎么动 | B看valid=true的消息 | A记什么 |
|---|---|---|
| 同一距离，往画面右边移 | x增大 | 右移正确/异常 |
| 同一距离，往画面下边移 | y增大 | 下移正确/异常 |
| 沿拍摄方向移远 | z增大 | 移远正确/异常 |
| 完全遮挡2秒，再拿开遮挡 | 不再有新有效观测；恢复后valid=true、时间更新 | 遮挡与恢复现象 |

完成后在终端2按Ctrl+C停止echo，再在终端3按Ctrl+C关闭刚才的预览。终端1可以继续运行。PnP的输入输出关系可在[第二课“PnP”](https://blog.secaumene.com/teach/class-2.html)对照。

## 第5步：B看雷达，按昨晚已经确认的全名查

这里不再让你们试一堆相似名字。昨晚确认的是：

```text
点云：/red_standard_robot1/livox/lidar
IMU： /red_standard_robot1/livox/imu
```

“点云”就是雷达扫到的一堆三维点；“IMU”提供加速度、角速度等数据。[MID360教程](https://blog.secaumene.com/kit/mid360-lidar.html)。

### 5.1 终端2查点云

```bash
ros2 topic info /red_standard_robot1/livox/lidar --verbose | tee "$LAB/lidar_info.txt"
```

**应该看到这两项：**

```text
Type: sensor_msgs/msg/PointCloud2
Publisher count: 1
```

命令末尾的tee会把同样输出存进本队txt，便于带回；不用手动创建这个文件。Type是消息格式；Publisher count是有几个程序声明在发它。昨晚的类型是PointCloud2。接着检查有没有真实消息：

```bash
ros2 topic echo /red_standard_robot1/livox/lidar --once --field header --qos-reliability best_effort 2>&1 | tee "$LAB/lidar_header_1.txt"
```

**应该看到类似：**

```text
stamp:
  sec: 1790000000
  nanosec: 123456789
frame_id: livox_frame
```

sec和nanosec合起来是时间戳；数字以现场为准。frame_id是这批点使用的坐标系名字；它只是名字，不会自动告诉你坐标轴在实物上朝哪边。它只打印点云的“信封信息”，不会把所有点刷满终端。

**下一步：终端2再输入：**

```bash
ros2 topic echo /red_standard_robot1/livox/lidar --once --field header --qos-reliability best_effort 2>&1 | tee "$LAB/lidar_header_2.txt"
```

两次时间应增加：先比较sec；sec相同时，第二次nanosec应更大。两次输出已经分别保存。

**如果不同：**没有话题、publisher为0、5秒收不到、时间不更新，Ctrl+C后把info和错误保存下来；今天不重开雷达驱动。类型变成别的也原样记录，不为了和教程一致去改驱动。

**本步正常后：**记录“点云有更新、PointCloud2”。不要接着直接启动旧默认Point-LIO：那个入口按CustomMsg配置，当前格式还没适配。这是下一项单独要接通的工作。

### 5.2 终端2查雷达IMU

C保持雷达静止，B输入：

```bash
python3 "$KIT/tools/read_imu.py" /red_standard_robot1/livox/imu 2>&1 | tee "$LAB/livox_imu_check.txt"
```

**应该类似昨晚：**

```text
已收到 30 条不同时间戳消息；frame_id=livox_frame
加速度模长中位数：0.99289；角速度模长中位数：0.01448
```

“模长”可以理解成把三个方向合成后的总大小，工具已经算好，不用你们手算。

- 收到连续消息、静止加速度数值约1：与g单位相符，记下原值，后续核对驱动单位。
- 若约9.8：可能按m/s²给数，记录，不能继续照搬按g配置的参数。
- 静止时三个分量不一定只有z有值，因为安装可能倾斜。
- 若四元数模长为1，也不能直接当可靠姿态；有些驱动填的是占位值。
- 没收到连续消息：保存输出；不把“话题存在”写成IMU已经通过。

做完这步就继续，不现场改雷达单位或网络参数。

## 第6步：串口是什么，B/C现在具体查什么

串口是NUC和云台控制板之间的一条数据通道，常经USB转接连接。控制板把IMU数据给NUC；桥接程序把它变成ROS话题。**设备上电，与ROS里收到云台IMU，是两件需要分别检查的事。** [串口桥教程](https://blog.secaumene.com/kit/bridge.html)。

`/dev/gimbal`是Linux里的设备路径，可能是指向真实串口的符号链接，可以把它理解为设备的快捷入口。

### 6.1 终端2检查链接

```bash
ls -l /dev/gimbal 2>&1 | tee "$LAB/gimbal_link.txt"
readlink -e /dev/gimbal 2>&1 | tee "$LAB/gimbal_target.txt"
stat -Lc '类型=%F 主次设备号=%t:%T' /dev/gimbal 2>&1 | tee "$LAB/gimbal_stat.txt"
```

**正常示例：**

```text
gimbal -> ttyUSB0
/dev/ttyUSB0
类型=character special file 主次设备号=...
```

ttyUSB0只是例子，现场可能是别的名字。它表示当前能找到这个字符设备，**不表示它一定就是目标云台**。

**如果链接能看到，但后两条没有目标或报不存在：**当前可能是悬空链接。保存三条输出，不执行ln -sf，不随便指定ttyUSB0。

昨晚曾显示串口“已独占”，说明按本地这版程序逻辑，当时已经成功打开TTY。今天检查的时间不同，链接状态可能不同。设备文件大小0也不能证明它没数据。

### 6.2 C看实物，B查询设备身份

```bash
ls -l /dev/serial/by-id 2>&1 | tee "$LAB/serial_by_id.txt"
udevadm info --query=property --name=/dev/gimbal 2>&1 | tee "$LAB/gimbal_identity.txt"
fuser -v /dev/gimbal 2>&1 | tee "$LAB/gimbal_owner.txt"
```

**B重点保存：**ID_VENDOR、ID_MODEL、ID_SERIAL、DEVLINKS这些行；fuser若有输出，保存占用进程号。C拍USB转接器、控制板型号和它实际连接到哪块板，帮助把屏幕上的USB身份与实物对上。

某命令不存在或没权限，保留错误即可。本步不改权限、不杀进程、不拔插网线，也不另开串口程序抢读。

### 6.3 看ROS里有没有云台IMU

```bash
ros2 topic info /red_standard_robot1/gimbal --verbose 2>&1 | tee "$LAB/gimbal_topic_info.txt"
```

若有publisher且Type为sensor_msgs/msg/Imu，继续：

```bash
python3 "$KIT/tools/read_imu.py" /red_standard_robot1/gimbal 2>&1 | tee "$LAB/gimbal_imu_check.txt"
```

**正常应有：**多个不同时间戳；静止角速度小而稳定；按统一协议，加速度模约9.8m/s²，四元数模接近1。有效读数还要结合协议定义核验。[统一协议](https://blog.secaumene.com/kit/protocol.html)。

**若没话题，或有publisher但8秒没有连续有效消息：**记录“云台反馈未接通”，把上面的设备身份和错误带回。今天不猜端口重开桥；雷达IMU不能替代云台IMU，视觉标定仍可继续。

**无论前面正常还是异常，终端2都执行一次：**

```bash
python3 "$KIT/tools/inspect_nuc.py" --expected-install "$T47_RUN/install" --output "$LAB/message_and_serial_check.json"
```

**应该看到：**生成message_and_serial_check.json的提示。工具只查询消息安装位置、设备链接和身份，不打开串口。里面若报告失败，原样带回，不根据它自动改设备路径。

### 机械零位、限位到底是什么？

- yaw是左右转，pitch是抬头低头。
- 零位是关节规定的0°姿态，要能重复找到。例如镜头朝车头、水平，只有在机械/控制定义确实如此时才算零位。贴线、定位孔或编码器归零方法可以提供依据。
- 限位是允许转到的范围。比如只能在某个角度内转，超出会碰结构或拉线缆；具体数必须来自这套设备，不能照抄例子。

C拍已有标线、安装结构和线缆走向，记录已有的零位/范围说明。不要转到撞头来找限位。IMU的yaw=0不自动等于关节机械零位。今天没有这些信息，也能先做相机标定。

## 第7步：本轮只核查G，不重画O/C

O/C的方向和换算已明确，记录为通过。G按单独的[G坐标定义核查与现场操作](G坐标定义核查_实际操作.md)执行：

- 先区分芯片原始轴、下位机输出轴、ROS的gimbal_link，核对安装旋转和打包路径。
- 顶部板目前只是疑似C型板，BMI088是该型号的候选器件；尚无现场烧录源码/版本，不据照片定轴。
- 上位机默认按线上xyz同轴发布；imu_axis_sign不改q，不用它试出“看起来正确”的方向。
- 需要实物时，只读已有云台IMU，用明确的yaw、pitch、低头yaw动作及参考姿态判据确认G；没有有效数据就保留未知。

专门教程已写明谁动哪个部件、B输入什么、a/w/q分别看什么，以及混轴时怎样记录。只补G的证据与标注。

## 第8步：相机原点、IMU原点找不到，尺子到底量哪里

相机原点是投影模型的光心，通常藏在镜头内部。镜头前玻璃不是已经确定的光心；内参K也不会告诉你光心离外壳多少毫米。[第二课相机模型](https://blog.secaumene.com/teach/class-2.html)。

IMU原点按设备/固件定义取参考点。电机中心、转轴交点、PCB角落都不能直接拿来冒充它。[统一协议的坐标定义](https://blog.secaumene.com/kit/protocol.html)。

### 8.1 A/C先各选一个能重复找到的外部点

- 相机选P：例如镜头安装肩面中心。拍照圈出是哪一个面/点。
- IMU选Q：有可信图纸就用标出的原点；没有就选模块安装板上明确的螺孔中心，拍照圈出。
- 记“P到光心偏移未知”“Q到IMU原点偏移未知”。今天先量机械基准初值，不把它填成精确外参。

### 8.2 G还没确定，就先在桌面建立测量方向H

C在桌面选一条明确直边，贴箭头指向一端，叫H的“前”；从上往下看，朝这端时的左侧叫H“左”；桌面竖直向上叫H“上”。拍照把这三个方向和P/Q都标出来。

**这只是尺量用的H，不是G。** 先把能量到的尺寸留下；后续有H与G方向关系再换算。

### 8.3 A读尺，C扶直角尺，分别量六个数

| 要量什么 | 具体做法 | 最后算什么 |
|---|---|---|
| 前后 | 在P、Q后方选一条与H前方向垂直的直边，作为共同的0线；尺从这条线沿H前方向读，分别记Q、P到这条线的距离 | P读数减Q读数；正数表示P在Q前面 |
| 左右 | 在P、Q右侧选一条沿H前方向的直边，作为共同的0线；尺从这条线沿H左方向读，分别记Q、P到这条线的距离 | P读数减Q读数；正数表示P在Q左边 |
| 高低 | 从同一桌面分别量Q高度、P高度，尺竖直 | P高度减Q高度 |

**具体摆尺：**比如量前后，把直角尺的一边贴紧“后方0线”，另一边指向H前。移动直角尺，让朝前的那条边经过Q，读Q的位置；再让它经过P，读P的位置。两次都从同一条0线读，不能分别从不同桌边量。左右测量换成右侧0线，做法相同；高低测量把直尺竖直立在同一桌面，读点的高度。点在空中时，用直角尺的横边对到点的高度，别把尺斜着量。

后方0线必须在两点后面、右侧0线必须在两点右边，才可直接按上表读正距离再相减。若桌边不合适，可在平整底板上画平行基准线；两次都用这条线。不要把P与Q的斜线总距离当作三个分量。

示例只说明读法：Q前后读30mm，P读110mm，前后差+80mm；P比Q偏右20mm，左右差-20mm；Q高100mm、P高150mm，高度差+50mm。记成H系(80,-20,50)mm，换米为(0.080,-0.020,0.050)。**这不是你们设备参数，也不是已经求出的G系外参。**

每项重新放尺量三次，保留原值。伸不进尺就量两点各自相对同一个能接近的基准再相减；连共同基准也没法重复定位，就拍照写明缺项，不估毫米数。

A的交付是一张表：P/Q定义、H方向、三次读数、单位、云台当时姿态、未知偏移。最终底盘还没固定的部件，今天不测最终底盘外参。

## 第9步：这次重新标定相机，B点程序，A/C摆棋盘

你已确认两份参数来自同一套设备且设置未变。重新采一轮用来核查约1.78倍焦距差异，先保留旧配置。

**第4步读数已经看完，就在终端1按Ctrl+C停止本队视觉，再在终端3标定。** 相机原驱动继续运行。这样减少同时运行的计算任务；之后需要重新看PnP时，再执行第4.2步。

### 9.1 A数棋盘、量单格，C保持板平整

内角点是棋盘内部四个格相交的位置。横向9个方格对应8个内角点；纵向7个方格对应6个内角点。数你们自己的板，不照抄8×6。

单格边长用打印后实物值。可以量连续4格总长再除以4，例如总长100mm，单格25mm。横纵都核对；板要平整、格子应为正方形，保留外围白边。[第二课“相机标定/Calibration”](https://blog.secaumene.com/teach/class-2.html)。

### 9.2 终端3确认标定器已安装

```bash
ros2 pkg prefix camera_calibration
```

**应该显示：**它的安装路径。若找不到，保存错误，这次先完成其它数据/照片，不在公共NUC安装依赖或重装ROS。

### 9.3 终端3输入实测规格并启动

```bash
CAL_DIR=$(mktemp -d "$LAB/calibration_XXXXXX")
read -r -p '横向内角点个数：' COLS
read -r -p '纵向内角点个数：' ROWS
read -r -p '单格实测边长，毫米：' SQUARE_MM
python3 -u "$KIT/tools/run_camera_calibration.py" --output-dir "$CAL_DIR" --cols "$COLS" --rows "$ROWS" --square-mm "$SQUARE_MM" 2>&1 | tee "$CAL_DIR/console.txt"
```

B按A数到的值输入。比如真是8×6内角点、25mm，就依次输入8、6、25。

**应该看到：**标定窗口、棋盘上的彩色角点连线、X/Y/Size/Skew进度条。保持默认普通针孔Pinhole模型。

- 有画面没连线：先查内角点数、棋盘完整、焦点和反光；C把板稍靠近、摆正再试。
- 没画面：核对/image_raw有publisher，终端已source。
- 程序报错：保留console.txt，不修改系统包。

### 9.4 C按顺序摆，A检查，B看Added sample计数

每个姿态稳住1—2秒，不连续晃。整板始终留在画面内：

1. 中间正对，整板约占画面宽度一半。
2. 移到左、右、上、下，再到四角，各保持几次。
3. 靠近、移远，角点仍需清楚。
4. 板中心大致不动，让左边靠近镜头、右边远离，再反过来。
5. 让上边靠近镜头、下边远离，再反过来。
6. 在不同位置再做几种倾角。

第4/5项像把书封面向一侧掀开一点，约20—40°作为起点；不是只把纸在画面里转成菱形。不同倾角和边角覆盖是标定质量的关键。[主办方标定采图要求](https://blog.secaumene.com/teach/class-2.html)。

终端`Added sample N`才是实际入选数量，近乎相同的画面可能不会新增。争取约40—60张有区别的有效样本，同时覆盖上述动作；不要只凑中央正视图。

### 9.5 B点CALIBRATE，计算后点SAVE

按钮可点击后，A确认姿态覆盖够了，B点CALIBRATE并等待。然后点SAVE，应该显示“本队SAVE完成：...”及本队目录。

这个入口沿用现成标定算法，只把SAVE改为本队目录新建文件，并禁用COMMIT；不会覆盖公共/tmp的标定包或写相机驱动。这里只用SAVE。

保存后终端3按Ctrl+C退出，再输入：

```bash
CAL_ARCHIVE=$(cat "$CAL_DIR/latest_saved.txt")
tar -tzf "$CAL_ARCHIVE" | sed -n '1,8p'
tar -xOf "$CAL_ARCHIVE" ost.yaml > "$CAL_DIR/ost.yaml"
cat "$CAL_DIR/ost.yaml"
```

**应该看到：**棋盘图left-xxxx.png、ost.yaml、ost.txt；YAML尺寸1440×1080，有K和D。不要预设fx必须接近999或1778。

某些版本不会打印总体重投影RMS，别找不存在的一行。把完整棋盘归档带回，我再算逐图残差。原始图的PnP要用K+D，不把校正后P混进去；低重投影误差还要配合独立测距。[OpenCV相机模型](https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html)。

### 9.6 留近、远两档验证图像

C换上黑边外轮廓边长100mm的平整ArUco码，A先用尺核对打印后边长。把码放在画面中间，大致正对相机。A沿镜头朝向量两个前后距离，例如约1m、2m：尺从8.1选定的相机P点量到码所在平面，记录具体基准和光心偏移是否未知。不要量成斜向的直线距离，也不要从NUC外壳起量；PnP的z是沿镜头朝向的深度。P到光心偏移未知时，保留原始尺量值，不能声称已经完成精确光心测距验收。

终端3输入：

```bash
BAG_DIR=$(mktemp -d "$LAB/validation_near_XXXXXX")
ros2 bag record -o "$BAG_DIR/bag" /image_raw /camera_info
```

录3—5秒按Ctrl+C。远档把validation_near换成validation_far再录。每次自动新建目录，不覆盖旧数据。/camera_info即使仍全零，也把原图与新标定一起带回；本次没有向驱动COMMIT。

## 第10步：离场前把结果带回Windows

### 10.1 B建一张记录，A/C把现场结果填进去

**终端2输入下面整段，包括最后的EOF：**

```bash
NOTE="$LAB/现场记录_$(date +%Y%m%d_%H%M%S).txt"
cat > "$NOTE" <<'EOF'
实验日期与参与人：
视觉：首次第4步已验证通过；本次是否发生重试异常=；异常文件=
O/C：方向及转换已明确，本轮不重测；G证据见专项记录=
雷达：点云时间有无增加=；IMU模长=；异常=
云台：/dev/gimbal实际目标=；USB型号/序列号=；云台IMU有无连续消息=
相机标定：横向内角点=；纵向内角点=；单格边长mm=；实际入选样本数=
近档验证：bag目录=；实测距离mm=；尺从哪个点量到哪个平面=；照片编号=
远档验证：bag目录=；实测距离mm=；尺从哪个点量到哪个平面=；照片编号=
P点具体位置=；P到光心偏移已知/未知=；照片编号=
Q点具体位置=；Q到IMU原点偏移已知/未知=；照片编号=
H前方向依据=；H左方向依据=；照片编号=
前后：共同0线位置=；第1次Q/P读数mm=；第2次Q/P=；第3次Q/P=
左右：共同0线位置=；第1次Q/P读数mm=；第2次Q/P=；第3次Q/P=
高低：共同桌面=；第1次Q/P读数mm=；第2次Q/P=；第3次Q/P=
H系P-Q三分量mm=；测量时云台姿态=
G原点与三轴已确认/未确认=；型号/固件/图纸依据=；照片编号=
机械零位/限位已有说明=；未知项=
未完成项与报错文件名=
手机照片保存在哪、由谁带回=
EOF
xdg-open "$NOTE"
```

**应该出现：**Ubuntu文本编辑器打开这张记录。A口述尺寸与标定规格，C口述型号/连接与照片编号，B填写，按Ctrl+S保存；没做的写“未做”，不知道的写“未知”。如果编辑器没有打开，在Ubuntu文件管理器进入第2步打印的本次记录目录，双击这张txt。

**手机照片不会自动进入NUC的结果包。** C把照片编号写进记录，再单独传到B的Windows一个文件夹，例如“20261017现场照片”；连同结果包一起带回。圈点/画轴请保存副本，原图保留。P/Q、前后左右基准、尺子读数要能在照片中对应上。

### 10.2 B停止本队程序，确认数据已经收尾

**B做：**终端1按Ctrl+C停止本队视觉，所有录包Ctrl+C收尾；不关闭原驱动。如果第9.6步录了验证包，**在刚才录包的同一个终端3**检查（BAG_DIR只在这个终端有定义）：

```bash
ros2 bag info "$BAG_DIR/bag"
```

应该有image_raw、camera_info和非零消息数。完整bag目录一起保存，不只取metadata.yaml。

### 10.3 B打包并下载

**终端2输入：**

```bash
REPORT_ID=$(date +%Y%m%d_%H%M%S)
RESULT="$T47_RUN/team47_onsite_$REPORT_ID.tar.gz"
tar -czf "$RESULT" -C "$T47_RUN/records" onsite_20261017
ls -lh "$RESULT"
```

应该显示生成的结果包和非零大小。它包含本次NUC记录目录中的txt、JSON、标定归档和bag；手机照片按10.1单独带回。通过NoMachine `Transfer a file → Download file from the server`下载到B的Windows；确认Windows真有这个文件再离场。[NoMachine下载说明](https://www.nomachine.com/sharing-files-between-computers)。

这次带回的重点：消息读取结果、串口身份输出、完整棋盘归档、近远原图、独立尺量与安装照片。标定未通过核验前，原程序参数继续保留。

## 附录A：只有本队源码/安装丢失时，才重新部署

昨晚程序已启动成功，通常跳过本附录。第2步提示找不到本队源码，或第3步确认本队安装不在时，才做。已有源码保留，下面不会覆盖它：

```bash
export KIT="$HOME/ws_championship/team47_transfer_20261017/team47_onsite_20261017"
mkdir -p ~/ws_championship/src
if [ -e ~/ws_championship/src/47-vision-check-20261016 ]; then
    printf '复用现有本队源码，不重复复制。\n'
else
    cp -a "$KIT/source_if_needed/47-vision-check-20261016" ~/ws_championship/src/
fi
source ~/ws_championship/src/47-vision-check-20261016/进入检查环境.bash
colcon --log-base "$T47_RUN/log" list --base-paths "$T47_SRC/src"
```

应列出gimbal_common、gimbal_interfaces、gimbal_perception、gimbal_tracker_bringup四包。只在四包和路径正确时继续：

```bash
CMAKE_BUILD_PARALLEL_LEVEL=2 colcon --log-base "$T47_RUN/log" build --base-paths "$T47_SRC/src" --packages-select gimbal_common gimbal_interfaces gimbal_perception gimbal_tracker_bringup --build-base "$T47_RUN/build" --install-base "$T47_RUN/install" --executor sequential --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release
source "$T47_SRC/进入检查环境.bash"
ros2 pkg prefix gimbal_perception
```

应看到Summary里4 packages finished，prefix在team47_check_20261016/install下。出现Failed/Aborted就保留第一条错误和Summary，别继续launch。然后回第2步给每个终端加载本次操作环境。

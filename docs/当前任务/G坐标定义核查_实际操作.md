# G（gimbal_link）：源码依据与现场判轴

更新：2026-10-17。本轮只处理G。用户已确认：昨天第4步首次全部通过，message type invalid发生在后续重试；O/C方向及转换关系已经明确，不再重复验证或绘制。

用户进一步确认：目前只有NUC的ROS2上位机工程，没有现用下位机源码或版本号；顶部板根据照片疑似RoboMaster C型板，尚不能确认板型、BMI088或该板是否就是当前IMU数据源。

## 这次去实验室，先做什么

**B阅读本教程；上传NUC只需[watch_gimbal_axes.py](../../tools/watch_gimbal_axes.py)，不用上传完整上机包，也不用重新编译视觉程序。**

1. **B先做第4节：**传这个py，查看话题，静止读8秒。
2. **收到连续数据：**A/C配合第5节的三个动作，B边看边记录。先满足文中供电和参考姿态条件。
3. **没有收到数据：**做第4.3节，把设备和报错记下来；跳过判轴动作，最后按第8节打包带回。

第1—3节解释依据，以及拿到固件资料以后查什么；目前没有源码，不用等它才能做只读检查。O/C和首次视觉已通过，本次不重画、不重测。

## 1. 目前能确定什么，还差哪一段

**G在协议中已有定义：它是随pitch级上的IMU一起运动的右手机体系，正轴按前、左、上约定。协议以C板在pitch级为前提；现场板型、数据来源及当前固件怎样实现这套轴还缺对应证据。** “上”是运动件在参考水平姿态下的上方；低头后G的Z也会倾斜，不会始终朝世界竖直方向。

| 层次 | 它是什么 | 现在掌握的依据 |
|---|---|---|
| S：IMU芯片原始系 | 芯片封装规定的感测轴 | BMI088是疑似C型板对应的候选；型号确认后才采用该器件的感测方向与PCB布局 |
| F：下位机输出系 | 固件经过安装旋转、偏置处理后，用来解算姿态和发送IMU的轴 | 协议要求三类数据统一到G；实际烧录固件的安装矩阵尚未取得 |
| G：ROS2 gimbal_link | 桥接器把线上数据放入这个frame | 本地C++桥不交换xyz；默认不翻轴；四元数只归一化、按ROS字段赋值 |

因此，正确追查顺序是：**芯片感测轴 → 固件安装旋转 → AHRS与发送字段 → ROS桥 → 实物动作核验。** C板的板边、接口朝向、云台外观本身不能补齐这条链。

### 本地源码已经查到的证据

以下路径都相对于本队仓库根目录：

| 文件、位置 | 结论 |
|---|---|
| `protocol/PROTOCOL.md` 第1节，约51—59行 | G随yaw、pitch运动；world是惯性参考，启动时的yaw零点不等于车头或比赛坐标 |
| 同文件§5.1，约237—241行 | 同一安装旋转应同时用于AHRS输入、回传accel、gyro |
| 同文件§6.2、§7 | q表示G到world的旋转；静止比力、三轴角速度均应在G表达 |
| `42-gimbal_serial/include/gimbal_serial/protocol.hpp`，`decodeImu()`约162行 | 按线上ax/ay/az、gx/gy/gz原顺序解码 |
| 同文件`imuLinearAcceleration()`、`imuAngularVelocity()`，约138、147行 | 每轴换单位、乘同轴sign；没有交换xy等安装旋转 |
| `42-gimbal_serial/include/gimbal_serial/imu_convert.hpp`，`makeImuMsg()`约32行 | q归一化；accel、gyro原轴放进ROS的xyz |
| 同文件`makeTf()`约63行 | world→G的平移写死为0，不是测出了IMU在空间的位置 |
| `42-gimbal_serial/config/bridge.yaml` | imu_axis_sign默认[1,1,1]；yaw_sign/pitch_sign只影响控制命令 |

Python桥的`gimbal_common/protocol.py:decode_imu()`与`vgimbal_bridge_node.py:_publish_imu()`同样没有替现场C板完成安装换轴。`frames.py`中把相机轴直接写成G/O关系的固定矩阵，注释注明来自虚拟配置中C/G外参全0；不能用它证明实物C与G同向。

**不要用imu_axis_sign试到“看着顺”为止：它不改q，可能把accel/gyro与姿态拆成两套轴。** yaw_sign、pitch_sign、电机极性也不提供G轴的证据。[协议在线入口](https://blog.secaumene.com/kit/protocol.html)；本次服务器未能访问，以上依据是仓库已有的v1.4协议和实际桥源码。

## 2. C型板资料告诉了什么

**若现场顶部板确是这个型号：**[大疆C型板用户手册](https://rm-static.djicdn.com/tem/35228/RoboMaster%20%20%E5%BC%80%E5%8F%91%E6%9D%BF%20C%20%E5%9E%8B%E7%94%A8%E6%88%B7%E6%89%8B%E5%86%8C.pdf#page=16)印刷页15—16说明，该型号板上的六轴器件为BMI088，原理图标为U1。STM32负责程序运行；它不是IMU。这是型号资料，不是对照片中这块板的最终鉴定。

[Bosch BMI088数据手册§8.3，印刷页56](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi088-ds001.pdf#page=56)给出芯片感测轴及正向。这只能定义S，不能直接定义固件输出F或ROS的G。现有资料没有给出“你们板上当前程序”的安装旋转。

一个明确的换轴例子来自[大疆官方C板例程的INS_task.c](https://github.com/RoboMaster/Development-Board-C-Examples/blob/master/20.standard_robot/application/INS_task.c)：

```text
BMI088_BOARD_INSTALL_SPIN_MATRIX
M = [ 0  1  0
     -1  0  0
      0  0  1 ]

输出(x,y,z) = (芯片y, -芯片x, 芯片z)
```

该例程的`imu_cali_slove()`对gyro和accel应用此矩阵，再送入姿态解算。**这是官方例程的实现示例，不是你们现有固件已经使用这个矩阵的证据。** 不据此在照片上定G轴，不下载或烧录这份例程。

本地两份比赛压缩包未找到`INS_task`、BMI088驱动或C板固件工程；已有教学工具README另指向`gimbal-edu-firmware`。对应服务器本次访问失败，不能声称已经读到它的安装矩阵。

## 3. 先取固件证据：B查文件，A核对矩阵，C对应安装

只查主办方提供、允许读取的固件副本，不进`~/ws_system`或`~/juger_runs`。

**B：**取得“现场板上所用版本”的源码或版本说明；在该固件工程根目录用编辑器全文搜索这些词。电脑已有rg时可输入：

这是**待主办方提供的信息**，目前不具备。拿不到完整源码时，取得“实际固件版本、芯片到输出轴的安装矩阵/轴向说明、accel/gyro/q是否统一映射、IMU原点参照”也有用。以下搜索命令只在拿到允许读取的工程后使用，不要求现在去NUC到处找。

```bash
rg -n 'BMI088_BOARD_INSTALL_SPIN_MATRIX|imu_cali_slove|gyro_scale_factor|accel_scale_factor|AHRS_update|INS_quat|0x01|install_rotation|mount_rotation' . -g '*.c' -g '*.cpp' -g '*.h' -g '*.hpp' -g '*.md'
```

**应该找到/交付的不是一个孤立矩阵，而是下列四项：**

1. 版本号/提交号，说明与现场烧录版本怎样对应。
2. BMI088原始gyro/accel经过哪个安装矩阵或轴交换函数。
3. AHRS拿到的是原始数据还是转换后的数据。
4. 0x01打包用的accel/gyro/q分别来自哪些变量，有没有再次调轴或求逆。

文件名可能不同；搜不到这些名称不等于没实现，应继续沿“读取传感器 → 姿态解算 → 串口打包”的调用关系查。没有这个工程就记“固件证据缺失”，不从NUC保护目录找，也不把网上例程当现用固件。

**A：**把安装矩阵写清为`v_F = R_F_S v_S`，逐行读出“F的x取S的哪一轴、是否负号”。核对accel、gyro和q所用轴是否一致；再结合芯片封装/PCB布局资料换成实际板边方向。若矩阵方向相反，先取逆，不能只看变量名猜。

**C：**先用已有照片找顶部板的明确型号/产品标签、固定点与pitch运动件的连接；沿通信线确认它连到NUC的哪条设备链。只有型号和数据源都对上后，才能把候选BMI088当本机器件。只补拍看不清的标签/固定点，不拆相机、不拆板壳去找芯片。若已取得固件方向，把“哪个板边/安装孔”与它对应。

若源码、版本和机械对应完整，可以先得出有依据的G轴；下面只做简短核验。若源码暂缺，下面能记录实际输出轴与物理动作的对应，不能据此声称补齐了芯片S→固件F的源码。

## 4. 必须实物验证时，先让B看见真正的云台数据

昨晚记录只证明雷达IMU有数据，云台桥没有收到合格0x01；当前若仍如此，不能判G轴。以下只订阅已有ROS数据，不新开串口桥。

### 4.1 传一个小工具，不重新构建视觉

下载本目录的[watch_gimbal_axes.py](../../tools/watch_gimbal_axes.py)，交给B。B仍在Windows开NoMachine，所有命令在远程NUC的Ubuntu终端输入。

```bash
source /opt/ros/humble/setup.bash
export ROS_LOCALHOST_ONLY=1
export G_DIR="$HOME/ws_championship/team47_check_20261016/records/g_axes_20261017"
export ROS_LOG_DIR="$G_DIR/ros_logs"
mkdir -p "$G_DIR" "$ROS_LOG_DIR"
printf '%s\n' "$G_DIR"
```

**应该显示：**`/home/实际用户名/ws_championship/team47_check_20261016/records/g_axes_20261017`。NoMachine菜单`Transfer a file → Upload file from the client`把这个py保存到打印的目录。[文件传输说明](https://www.nomachine.com/sharing-files-between-computers)。

传完输入：

```bash
ls -lh "$G_DIR/watch_gimbal_axes.py" 2>&1 | tee "$G_DIR/tool_file.txt"
ros2 topic info /red_standard_robot1/gimbal --verbose 2>&1 | tee "$G_DIR/gimbal_topic.txt"
```

**应该看到：**py存在且非0字节；话题Type为sensor_msgs/msg/Imu、Publisher count为1。命令末尾的tee把屏幕上的字同时存进本队文件，不用另建txt。

若这个话题不存在，B输入：

```bash
ros2 topic info /offboardlink/imu_data --verbose 2>&1 | tee "$G_DIR/offboardlink_topic.txt"
```

这是协议默认名。只在它确有现用发布者时使用它，并在后面的--topic中统一替换。

两个话题都没有数据来源，先记录“云台IMU链未通”，带回桥接日志及设备身份；本轮不做判轴动作。Publisher存在仍需下一步收到不断更新的真实消息。多个发布者先保存名单，不把混合数据当同一块板。

若发布者是C++的`/robo_bridge`，B再查：

```bash
ros2 param get /robo_bridge imu_axis_sign 2>&1 | tee "$G_DIR/axis_sign.txt"
```

应为[1,1,1]。节点名按topic info的实际Node name与Node namespace合成；不同就用那个完整名字。不设参数。若参数不存在或不是[1,1,1]，记录桥版本及当前值；此时ROS输出与线上原轴不一定相同，后续数据只记录，不直接认作协议G。

### 4.2 先静止8秒，认清输出

```bash
python3 "$G_DIR/watch_gimbal_axes.py" --topic /red_standard_robot1/gimbal --output-dir "$G_DIR/data" --label static --seconds 8 2>&1 | tee -a "$G_DIR/static_console.txt"
```

**应该每秒显示约4行，例如：**

```text
a=(+0.010, -0.020, +9.800) m/s2  w=(+0.001, +0.002, -0.001) rad/s  roll=+0.1 pitch=-0.1 yaw=+12.0 deg
```

每个括号都按x、y、z排列。a是加速度计比力，w是角速度；roll/pitch/yaw由收到的q换算，是惯性姿态角，不是机械编码器角。示例不是你们必须出现的数。

工具把每条显示样本连同frame、stamp、q存成新的JSONL，不覆盖旧文件。它不打开串口、不发电机命令、不发TF、不装依赖。它也不能证明q来自正确AHRS，只是方便对照。

**如果不同：**

- 一直“尚未收到新IMU数据”：停止判轴，查现有云台链；别用雷达IMU代替。
- frame不是gimbal_link、stamp为0或不更新：保存输出，先查发布者/时间链。
- 明明转了但w和q一直不变：不能当真实姿态通过，应检查数据来源/固件。
- 静止时a模长明显不是约9.8：记录原值与单位，不能直接套用后面的m/s²示例。

### 4.3 没有云台数据时，B把这些结果存下来

仍在刚才那个NUC终端，输入：

```bash
ls -l /dev/gimbal 2>&1 | tee "$G_DIR/gimbal_link.txt"
readlink -e /dev/gimbal 2>&1 | tee "$G_DIR/gimbal_target.txt"
stat -Lc '类型=%F 主次设备号=%t:%T' /dev/gimbal 2>&1 | tee "$G_DIR/gimbal_device_type.txt"
udevadm info --query=property --name=/dev/gimbal 2>&1 | tee "$G_DIR/gimbal_usb_identity.txt"
fuser -v /dev/gimbal 2>&1 | tee "$G_DIR/gimbal_owner.txt"
```

**正常时可能看到：**链接指向某个tty设备，类型为character special file；USB身份里有ID_MODEL、ID_SERIAL等，fuser可能列出占用进程。ttyUSB0只是可能的名字，不是让你们照着指定它。

**报不存在、无权限或没有输出：**也保留文件。它们说明当前状态，不要为得到“正常输出”去换端口、改权限或杀进程。这个检查不打开串口；字符设备存在也不等于接对了板或收到了IMU。

本步结束后，A/C只补看不清的板型标签、连接和固定点；跳过第5节动作，按第8节把这次记录带回。

## 5. 三个动作，分别确定Z、Y、X

**动作前：**按实验室现有方式停止控制，电机不供电，C板及通信保留供电；只发零速度不等于电机已断电。若现有供电不能把两者分开，让现场设备负责人处理供电，不随机拔线。不能保持板上电又让关节自由活动时，本轮只做资料核查，不硬掰电机。协议§13也采用电机不上电的手动核验。

整个测试只用两个现有关节，轻转约10—15°，在已有活动余量内，不找限位。C动机械支架，A看动作并记方向，B看三轴和姿态。先让底座水平、镜头大致水平，作为统一的物理参考姿态；**镜头水平不保证G水平，必须结合下述数据核对。**

### 5.1 Z：水平姿态下，只转yaw

B输入：

```bash
python3 "$G_DIR/watch_gimbal_axes.py" --topic /red_standard_robot1/gimbal --output-dir "$G_DIR/data" --label yaw_level --seconds 30 2>&1 | tee -a "$G_DIR/yaw_level_console.txt"
```

看到数据后，C从上往下看，让上层云台向逆时针方向转10—15°，约1—2秒转完；停2秒，再顺时针回去。重复3次。C板必须随这个运动件一起转。

**协议对应正确时：**逆时针阶段w的z为正、反转为负，x/y明显较小；q换出的yaw随逆时针增加。停住时w回到静止基线附近。

**据此得出：**在这个参考姿态下，G的+Z沿yaw轴朝上。若只有另一个轴明显变化、符号相反或多轴明显混合，保存全部xyz，不把最大的那个分量直接改名成z；可能存在安装旋转、参考姿态或发布源问题。

**进入下面两步前再检查参考姿态：**静止时q的roll/pitch接近0，a约(0,0,+9.8)，并且本步yaw对应正z。三者相互吻合才按后面的纯轴表继续验收。这里只做粗判，不把小偏差当精密标定；若不吻合，记“G参考姿态/安装关系未定”，后面可以保留动作数据，但不套表判通过或直接认定固件错误。

### 5.2 Y：保持yaw不动，只做低头

B输入：

```bash
python3 "$G_DIR/watch_gimbal_axes.py" --topic /red_standard_robot1/gimbal --output-dir "$G_DIR/data" --label pitch_down --seconds 30 2>&1 | tee -a "$G_DIR/pitch_down_console.txt"
```

C扶住yaw级，让pitch运动件缓慢低头10—15°；停2秒，再抬回。重复3次。不是只移动码，也不是把相机从支架上拧松。

**已满足参考姿态条件且G的Y与pitch轴对应时：**低头阶段w的y为正，抬回为负，x/z明显较小；q的pitch增加。低头停住时，a的x应较水平时变负，z仍为正。例如实际G相对水平低头约15°时，静止a约(-2.54,0,9.47)m/s²。这里的15°指实际G倾角，不是从任意初始姿态又转了15°。

**据此得出：**G的+Y沿pitch轴、正转方向使运动件低头；结合上一步的+Z和右手关系，能把+Y标到支架的实际一侧。不要仅因某板边在照片左侧就叫它+Y。

**怎样在两端选出+Y那一端：**右手四指顺着刚才“低头”的转动方向弯曲，拇指指向的pitch轴端就是+Y。C指出它靠近哪个固定孔/结构，A记下；这是用动作定方向。

### 5.3 X：保持低头，再转yaw；不需要roll电机

B输入：

```bash
python3 "$G_DIR/watch_gimbal_axes.py" --topic /red_standard_robot1/gimbal --output-dir "$G_DIR/data" --label yaw_while_down --seconds 30 2>&1 | tee -a "$G_DIR/yaw_while_down_console.txt"
```

C先从已核对的参考姿态再次缓慢低头10—15°，保持住，静止2秒；然后仍按“从上看逆时针”的方向转yaw，再反向回去，重复3次，尽量不同时改变pitch。A记下q显示的实际G的pitch，不用手动增量代替。

**已满足参考姿态与关节轴条件、G实际pitch为小正角时：**

| 动作 | w.x | w.y | w.z |
|---|---:|---:|---:|
| 低头状态，yaw逆时针 | 负 | 接近静止基线 | 正 |
| 低头状态，yaw顺时针 | 正 | 接近静止基线 | 负 |

例如yaw角速度+0.20rad/s、G低头15°时，w约(-0.052,0,+0.193)rad/s。负的x是正常的三维投影：低头后G也倾斜了，竖直的yaw转动会同时落在G的负X和正Z上。

依据协议§6.2：`w = (-yaw速度·sin(pitch), pitch速度, yaw速度·cos(pitch))`。A可用记录核对`-w.x/w.z ≈ tan(pitch)`；这里pitch是G相对水平的实际倾角。只在上述轴关系成立、pitch保持不变、底座水平、远离零角和奇异姿态时用这条比值。它用于符号/粗方向核查，不是精密标定。

**最终+X的实物方向由+Y×+Z确定，再由本步x分量和5.2的静止a.x做交叉核验。** 三步一致后回到同一参考姿态，在已有照片副本上标G并写明照片姿态；不再画O/C。不要求G+X天然与相机光轴完全平行。

现场比手势时，让右手食指沿已确认的+Y、中指沿+Z，拇指所指就是+X。先在同一参考姿态对应到支架，再注明靠近哪条机械边；不把该方向直接当成相机外参为0。

## 6. 怎样判定通过，怎样处理不一致

这次的“通过”是确认输出轴的方向，不是证明完整惯导精度或全部固件正确。

- 三种动作各重复3次；运动时符号可重复，反向动作符号反转，停住回到静止基线。不要按一帧噪声判断。
- 在近水平的Z/Y独立动作里，预期主轴的变化应明显大于静止噪声和其它分量。若多轴相当，不强行按最大值定轴。
- 陀螺、q的变化、静止比力互相一致。静止时协议预测是`a_G ≈ R_world_G(q)^T·(0,0,9.80665)`；小工具已把该预测也写入JSONL供带回检查。
- 记录动作方向、当时pitch状态、板子是否一起转、实际发布者和轴sign。yaw经过±180°时会换显示范围，不能把+179°到-179°误判为突然反转。

**混轴时怎么办：**保留同一次试验的全部xyz和q，核查安装矩阵/参考姿态；必要时再计算完整安装旋转，不在PC侧随意取负或交换轴。如果陀螺方向正确而q/比力不一致，要查固件是否对三类数据用了同一映射。

## 7. G原点仍需要什么

G的轴方向与G原点的毫米位置是两项问题。协议和描述文件只把原点放在IMU，未给出相对安装孔的机械尺寸；桥的TF平移0也不能定位芯片。

先确认实际IMU型号，再取对应PCB/安装图上的器件位置。若确为BMI088，后续可以讨论用其封装中心作为工程参考原点，并记录这个约定的近似误差；目前不先认定芯片型号。不能把电机转轴交点、板中心或螺孔中心直接当作已确认的IMU原点。轴向核验与原点位置分开记录，最终相机—G平移等依据齐全后再定。装车前不做最终底盘外参。

## 8. A/B/C交付与带回结果

**A：**一张G轴证据表：S→F矩阵或“缺失”、F→ROS映射、三种动作的实际符号、最终G三轴箭头依据、原点未知项。

**B：**现用固件/桥版本与发布者信息；`$G_DIR/data/`里四组记录（静止、水平yaw、pitch、低头yaw）。如果没收到消息，交付失败输出，不写通过。

**C：**沿用已有安装照片，对应C板固定点及运动件；只补必要的型号近照、带动作名称的短视频或照片编号。

B最后在同一个NUC终端输入：

```bash
G_ID=$(date +%Y%m%d_%H%M%S)
G_OUT="$HOME/ws_championship/team47_check_20261016/records/team47_g_axes_$G_ID.tar.gz"
tar -czf "$G_OUT" -C "$G_DIR" .
ls -lh "$G_OUT"
```

通过NoMachine下载显示的tar.gz；手机视频/照片另传，并注明对应哪个动作。没有自动操作硬件或修改任何轴参数。

## 9. 这轮实际完成的离线核验

1. 生产C++协议/串口代码：44组黄金帧、5种流式分片、10万噪声字节及OS新建伪终端通过，启用ASan/UBSan；伪终端测试是115200，不代表真实921600链路通过。
2. 新增G专项8项：单位姿态、roll/pitch/yaw已知姿态、等价四元数、无效姿态拒绝、生产Python解码的静止比力与q一致、低头yaw/独立pitch角速度关系，以及错误翻轴反例，全部通过。
3. 黄金帧的静止低头例：原accel与q预测相差约0.0022m/s²；仅翻accel.x而不改q，残差变为约3.895m/s²。这是软件反例，不是本机测量。

复跑入口为仓库的`tools/verify_gimbal_axes_offline.py`与`tools/verify_protocol_offline.py`，使用个人电脑已有的隔离环境。此次未执行整套375项回归，不把局部复跑写成全系统重新验收。

**仍未确认：**顶部板型号、实际芯片、NUC串口对应哪块板、现场固件版本/安装矩阵、真实G轴和原点。只读显示工具的实际ROS通信也须现场验证。无烧录、无真实设备打开、无真实电机动作。

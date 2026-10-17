# 赛题一：A、B、C实验室执行手册

更新：2026-10-16。队号暂用47。代码基线为02d6ef2，本批离线开发修改以交付清单为准。配套[现场记录表](现场记录表_ABC.md)记录实际结果，保留空白模板。

**当前硬件状态：相机与云台已固定，整套装置还没有安装到底盘。**现在测相机与云台的刚性关系、检查真实图像和反馈；底盘/雷达/云台之间的最终外参，等对应部件固定后再测。下文涉及整车的A5/A6、C3运动验证和第六部分，都有这项前提。

本批已做实际离线开发和验证，没有操作NUC或真实电机。Mac上的OpenCV/模拟消息/伪终端验证不能代替Humble、DDS、真实串口、真实定位和闭环验收。

**本次只按[G操作教程](G坐标定义核查_实际操作.md)执行。下文的完整流程保留为后续参考；首次视觉和O/C不重复操作。**

### 本轮只处理G（用户补充）

首次视觉第4步已通过，message type invalid仅在后续重试出现；O/C轴向及转换关系已明确，不重复安排。当前执行[G专项教程](G坐标定义核查_实际操作.md)。现用下位机源码/版本尚缺，板型也仅疑似C型；以下旧分工与整车流程保留作参考。

### 2026-10-17现场反馈更新

昨晚专用视觉已在NUC启动，识别63帧、PnP有效44帧/拒绝19帧；雷达IMU收到30个不同stamp。后续若再次出现消息错误，可按[完整上机教程](现场上机教程_ABC.md)恢复查看终端的自定义消息、做两份K的独立测距对照、查/dev/gimbal身份。点云实际为`/red_standard_robot1/livox/lidar`的PointCloud2，IMU实际为`/red_standard_robot1/livox/imu`；不照旧默认短名启动CustomMsg版Point-LIO。B现场仍使用Windows的NoMachine，所有实机命令在远程NUC执行。

### 之前现场轮次的分工（保留参考）

1. **B：按[现场上机教程](现场上机教程_ABC.md)在Windows的NoMachine里恢复本队消息环境，保持视觉运行再读输出。**不重做已成功的四包部署；类型仍失败则保留完整异常。
2. **A：独立量码黑边和近远距离，核对原始图设置；与B/C完成同一批角点下的两份内参对照。**不直接替换当前标定，不用重投影小代替尺量验收。
3. **C：拍IMU/相机/镜头型号与安装关系，配合尺量；B只读查询/dev/gimbal的目标、身份和占用。**不凭猜测换串口，不为了补坐标图把未知G轴设为相机轴。

B个人Ubuntu上的20拍/1800拍合成提交实验仍可另行推进；本轮实机时间按前面的G专项安排使用。完整测量/接口方法继续参考下文，当前实际话题全名以现场复查为准。

### 软件已做到哪里，暂时不能把什么当作通过

| 模块 | 本批实际结果 | 仍须现场确认 |
|---|---|---|
| 检测/PnP | 修复OpenCV返回值、角点ID/图像stamp绑定、丢失处理、平面PnP候选；真实OpenCV合成码测试通过 | NUC的OpenCV版本、曝光/焦点、真实码尺寸与测距 |
| 坐标/预测 | 完整相机TF按图像stamp查询；滤波在观测时刻更新；CSV再采样到tick/tick+0.1；缺TF不拿单位阵替代 | 实测TF、传感器时间基准、真实预测误差 |
| 底盘/地图 | 全三维传感器杠杆臂换到底盘中心；保留源stamp；地图保留分辨率、尺寸、origin yaw | Point-LIO真实输入/漂移、自有OccupancyGrid与地图对齐 |
| 云台 | 两轴完整运动学正反解及同步接口已实现；控制律有离线回归，默认禁止控制 | 机械零位/限位、实测安装、真实关节或标定派生关节、闭环性能 |
| 通信 | 生产C++解析器与串口类通过黄金帧、分片、噪声、复位/回绕、OS伪终端和ASan/UBSan | 完整ROS桥编译、真实921600链路、固件停止行为 |
| 提交/评分 | 九列逐拍flush、重启去重/损坏拒绝/单写者；按正式权重计入底盘40%；1800拍合成链路通过 | NUC独立观察器<60ms团队门槛；官方80ms作废线；真实精度 |

合成链路有40ms图像延迟、0.1px角点噪声和0.8秒遮挡；1800拍中第0拍及605—608拍如实无效。剔除明确的初始化/恢复窗口后，当前位置x/y RMSE约2.82/0.69mm，100ms预测约3.96/1.47mm；**这是已知理想几何的程序核验，不是实机精度指标**。完整合成评分仍包含全部1800拍。

## 一、这轮要拿到什么，先后怎样安排

| 负责人 | 这轮实际工作 | 交付 |
|---|---|---|
| A | 先确认相机/云台刚性关系和PnP误差；整车固定后再量完整链 | 有轴向图、测量依据和误差的变换表；计算与实物对照 |
| B | 构建现有核心包，接通视觉、只接收云台反馈、验证提交接口 | 实际版本和构建记录、启动命令、输入输出和提交产物 |
| C | 确认最终装配、端口、零位和实际话题；与A测量；检查LIO输入 | 安装照片、原始尺寸、设备表、传感器与定位记录 |

**并行安排：**B先在个人Ubuntu做无硬件的20拍提交实验；A/C同时确认实物坐标。之后在NUC汇合做静态视觉、反馈；条件齐全才单独验证LIO。完整坐标链通过后才进入运动控制。

粗略时间：环境和版本20—40分钟；物理坐标及机械初测60—120分钟；静态视觉30—60分钟；反馈20—40分钟；提交实验约20—40分钟，1800拍本身约3分钟。LIO和地图接入时间取决于输入、依赖和安装信息，不承诺当场完成。

### 当前已经知道与尚未做到

- 已有检测、PnP、估计、方向建议、串口、CSV及地图保存源码，部分接口已修复；不等于整套系统运行正确。
- 已有1440×1080的相机K/D，继续使用，不无理由重标定。`/camera_info`的全零K不能用于PnP。
- 赛题一确实要控制yaw/pitch跟踪指定ArUco，还要定位、100ms预测、建议方向、地图和九列CSV。ID是码的编号，不是印刷数字OCR；本队按QAF做赛前运行时选择，不照搬赛题二的`/judge/target_id`流程。
- 完整安装外参、IMU世界与LIO对齐、关节零位/限位、观测时间接入、真实地图链和实机精度仍待完成或验证。
- 原`competition.launch.py`仍含旧串口和未确认假设，本轮不用它直接启动实机整链。

规则依据：[官方环境说明](https://blog.secaumene.com/kit/campus-contest-env.html)、用户确认有效的任务书截图和QAF。以下实验门槛是团队排错标准，标明之处不属于官方评分标准。

## 二、出发和到场准备：三个人共用

### 2.1 带什么，向现场确认什么

C带卷尺、直尺、直角尺；有卡尺/水平仪可带。准备可移除胶带、笔、纸和拍照工具。A带手册、记录表；B带本批完整源码和交付清单，不只带文档。

到场先确认并写下答案：

1. 哪个是最终比赛装配？雷达最终固定在哪里？桌上散放设备的照片不能作为最终外参。
2. 底盘中心和车头如何定义？机械零位怎样识别？允许触碰/移动哪些部分？停止异常设备的现场流程是什么？
3. 哪块是输出云台姿态的IMU？它的参考原点在哪里？**固件输出的`gimbal_link`的+x、+y、+z在实物上分别朝哪里？**请现场指认，不只问“IMU在哪”。
4. 相机、雷达、串口目前各由哪个程序打开？已有正确数据时先复用，不启动第二份驱动。

不拆相机、镜头、IMU或雷达来找中心；不强扳带电电机。需要改变设备状态时，按管理员允许的办法做。

### 2.2 在哪台机器执行

| 环境 | 本轮用途 |
|---|---|
| Mac | 编辑、看源码、整理数据；不能把本机离线结果当Humble/硬件验收 |
| 个人Ubuntu22.04/Humble | 构建核心包、合成提交实验 |
| NoMachine里的NUC终端 | 访问NUC传感器，做视觉、反馈、LIO检查 |

`ROS_LOCALHOST_ONLY=1`意味着个人Ubuntu不能因为与NUC同局域网就直接收到其话题。实机命令在NUC运行；实机所有终端须与驱动使用同一ROS域。

每个实机新终端，在工作区已构建后执行：

```bash
source /opt/ros/humble/setup.bash
source ~/ws_championship/install/setup.bash
export ROS_LOCALHOST_ONLY=1
printenv ROS_DISTRO
printenv ROS_LOCALHOST_ONLY
printenv ROS_DOMAIN_ID
```

前两项应为`humble、1`。域为空通常表示默认0，但仍应向驱动启动者核对。尚无`install/setup.bash`先做B1，不反复source不存在的文件。

提交合成实验另用隔离域87（仅示例），结束后关闭测试终端。**不要把87带回实机采集终端。**换环境后CLI仍显示旧状态，可以在新环境执行`ros2 daemon stop`后再查。

### 2.3 每项怎样留记录

三人共用一个实验编号，例如`lab_001`。在个人目录或选手工作区保存记录表、安装照片、版本/构建日志、指定话题bag、CSV/地图和观察器结果。重做就换编号，不覆盖。

每项写：操作人、时间、机器、ROS域、装配/云台状态、代码及改动、参数、命令、实际结果、判断、下一步。只有“检查过”或成功截图不足以交接。

不读取/复制/修改`~/ws_system、~/juger_runs`；不向`/judge/*`发布；不随意改IP、拔插既有网线或修改系统时间；不录制全部话题。本手册只使用明确列出的选手/允许输入。

## 三、A：把实物关系变成可验证的坐标变换

### A0. 先学什么，用来解决什么

| 资料 | 章节 | 本次用途 |
|---|---|---|
| [第一课](https://blog.secaumene.com/teach/class-1.html) | “齐次坐标系/Homogeneous”“旋转矩阵” | 解释平移方向、矩阵三列和变换顺序 |
| 同上 | “欧拉角/RPY”“四元数/Quaternion”“TF2” | 区分安装角、IMU世界姿态与关节角，理解按观测时间查姿态 |
| [第二课](https://blog.secaumene.com/teach/class-2.html) | “针孔相机模型/Pinhole”“畸变/Distortion”“PnP”“误差分析” | 理解PnP坐标、测距和误差来源 |
| 仓库`gimbal-edu-tools/gimbal_edu_description/README.md` | “TF树”“外参yaml”“量取方法” | 核对套件实际采用的相机/IMU定义 |

可以边做边学，必要原理本人理解；暂不需要先学完李群、重写AHRS或BA。

### A1. 第一步：说清是哪两个坐标系

先只做“云台IMU—相机”，再扩到底盘和雷达。统一记号：

\[
p_A=R_{A\leftarrow B}p_B+t_{A\leftarrow B}.
\]

`T_A←B`把B系坐标变成A系坐标；`t_A←B`是B原点在A系中的坐标。逆变换为`Rᵀ、-Rᵀt`，不能简单都取负。

| 简记 | 原点 | 正方向 |
|---|---|---|
| B：底盘系 | 主办方定义的底盘中心 | x车头、y左、z上 |
| G：`gimbal_link` | 云台IMU参考原点 | 固件映射后的x前、y左、z上，须对应到实物 |
| C：`camera_link` | 投影模型的相机光心 | x沿光轴前、y图像左、z图像上 |
| O：`camera_optical_frame` | 与C相同 | x图像右、y图像下、z沿光轴前 |

相机“上”指图像上；安装有滚转时不一定朝重力反方向。相机内参K描述投影，不给IMU到相机的安装关系。

需要分清两件事：`C←O`是已知坐标约定，不用测；`G←C`是实际安装，需要确认和测量。“相机装在云台上”不能推出零外参。

### A2. 到实物面前，怎样找原点和轴

由C确认可接近测量后，站在相机后方、面朝拍摄方向；下面左右均按这个视角，避免面对镜头时看反。

1. 请现场指认IMU，或按可信装配图确认。拍一张同时能看见IMU位置、相机和yaw/pitch结构的照片，在副本上标G原点和三轴。看不到芯片时，记录可见安装基准及到IMU原点的偏移；不能随便选PCB角点当原点。
2. 按固件说明/现场指认标G轴。芯片手册轴和导出轴可能不同：固件已对accel、gyro、q做一致映射，不能用丝印猜一套轴覆盖它。
3. 找相机光轴及机械参考。镜头前端不是光心，外壳边也未必精确平行光轴。优先查厂家机械图/镜头基准；没有光心位置时，选可重复定位的参考点P，并写清“P到光心偏移未知/估计多少”，不偷偷填0。
4. B显示原始图像，C在镜头前移动纸目标。按相机后方视角向右、向上移动，图像也应分别向右、向上。若相反，记录驱动/显示是否镜像或旋转；不要把显示镜像当机械安装旋转。
5. 看IMU与相机是否在同一块随pitch运动的刚性部件上，请管理员说明哪些部件随yaw、哪些随pitch。若IMU在底座、相机在俯仰臂，`G←C`不是常量，先画实际运动链。

合格产物：标好原点/轴的照片或草图，每个方向有依据。若G轴或原点不明，写成具体待确认问题；A仍可先做A6/A8的数学核验，不必停下所有工作。

### A3. 平移怎么量：量三个分量，不量斜线

目标为`t_G←C=[前后,左右,上下]`，用G轴表达。

1. 保持相机/IMU同一姿态，记照片号和关节状态。
2. 以G原点或已知代理点为起点，用直角尺将相机光心参考投影到G的x、y、z方向，分别量三个距离。可用共同安装面、直角尺、铅垂线辅助；斜线长度不等于任一坐标分量。
3. 难直接量时用共同基准相减。例如同一水平面到两点的高度为hC、hG，竖直差为hC-hG。**若G_z不竖直，这个竖直差不能直接当G_z分量。**
4. 每项量三次，保留mm原值、均值、读数波动、光心代理误差。精度来自量具和基准，不随意写“毫米级标定”。
5. A换算米：80mm=0.080m。前/左/上为正，后/右/下为负。

例：相机在IMU前80mm、右20mm、上50mm，则`t_G←C=(0.080,-0.020,0.050)m`。**这是符号例子，不是本机参数。**

光心位置未知时，交“机械初值＋未知偏移范围”，再用A7检验；不能宣称外参标定完成。

### A4. 旋转怎么确定：光轴方向还不够

旋转矩阵三列分别是C的x/y/z单位轴在G里的方向。所以必须同时确认光轴和图像上方向；只确认镜头朝前，仍不能确定滚转。

- 有可信图纸/平行机械基准，且C_x/C_y/C_z分别与G_x/G_y/G_z同向，可将`R_G←C=I`作为有依据的安装初值；肉眼“差不多”不等于精确零角度。
- 有倾角时，选G上已知的两条不平行基准，与相机光轴/图像上方向关联的机械基准比较。用角度工具量的是**两者相对方向**；只量相机对桌面的角度，不能得到相机对IMU的角度。
- A先画轴/写旋转矩阵，B再按统一`R=Rz(yaw)Ry(pitch)Rx(roll)`转配置；顺序roll,pitch,yaw，单位rad。正yaw把+x转向+y，正pitch把+x转向-z。不要靠试正负调到看着对。
- 有测量方向向量时，检查单位长度、正交性、右手性，要求`RᵀR≈I、det(R)≈1`。三条不正交测量向量不能直接当精确旋转矩阵。

缺G的方向基准、光轴关联或需要更高精度时，要补独立标定数据。先做A7的共同基准位置实验，再决定是否需要进一步标定。**一张ArUco图只给码相对相机的位置；一个IMU四元数只给姿态；它们不能凭空给出IMU—相机平移。**

已知的光学约定是：

\[
R_{C\leftarrow O}=\begin{bmatrix}0&0&1\\-1&0&0\\0&-1&0\end{bmatrix},\qquad p_C=(z_O,-x_O,-y_O).
\]

完整关系为：

\[
p_G=R_{G\leftarrow C}R_{C\leftarrow O}p_O+t_{G\leftarrow C}.
\]

教学URDF的固定光学关节只给`C←O`，不替你测`G←C`。假设安装同向并使用A3的虚构平移，`p_O=(0.10,0.05,2.00)m`应算成`p_G=(2.08,-0.12,0.00)m`；图像右下对应G系偏右、偏下，方向应能解释。

### A5. 底盘、雷达、转轴按同样标准记录

画链：`底盘B → yaw基准 → pitch基准 → IMU G → 相机C → 光学O`，另有`底盘B → 雷达参考S`。L留给定位系使用，不复用同一字母。

1. 雷达最终与底盘固定后，`T_B←S`是常量。原点/轴用厂家参考或驱动定义，不能以玻璃罩顶点替代。
2. 转轴是一条线，不天然规定线上哪一点是原点。选一个可复现安装面交点等作为参考，写清依据。
3. 每条平移用**父坐标系的轴**表示，不把不同关节的“前左上”统统当底盘轴。
4. 关节转动时相机相对底盘变化；某一姿态量出的`T_B←G`不能当全程常量。
5. IMU不在转轴交点时，转动还带来原点平移；四元数本身不能补杠杆臂。

A交B的每条边须包含：父/子、原点和轴、固定/变化、数值/单位、来源/误差、测量时关节状态和未知项。不能只交五组无方向的尺寸。

### A6. IMU世界、LIO和比赛起点，现场如何对齐

L表示自有定位系，W表示云台IMU惯性系，M表示比赛起点系。不能因都叫world/map就当相同。

协议q满足`v_W=R(q)v_G`。TF边名`world→gimbal_link`指父子关系，不是让你把这个点变换取逆。就绪时yaw=0是IMU参考，不等于云台朝车头；固件复位后需重新对齐。

现场流程：

1. A/C确认一个可复现、方向已知的云台相对底盘姿态，写下机械零位/关节角依据。只说“看着朝前”不够。
2. 整套装置静止，B同时录自有LIO和云台IMU约5—10秒，保留stamp，A/C记实际姿态。不能拿不同时刻的两条echo当同步证据。
3. 先把LIO参考点换到底盘；结合已知机械姿态得到`R_L←G=R_L←B R_B←G`。
4. 取同一静止窗口有效的`R_W←G`，求`R_L←W=R_L←G (R_W←G)ᵀ`。这里只解旋转，平移还须用原点与运动链。
5. 用另一个已知姿态独立核验；静止窗口减少时间误差，不代表传感器时延已标定。

若第1步缺失，公式不能生成可靠结果。重力能约束倾斜，不能单独给绝对yaw；应补零位/相对朝向依据或另设计标定实验，不把默认`world_odom_xyz_rpy`全0当答案。

比赛起点：取**起点时刻底盘中心**的`p0,ψ0`，使用：

\[
p_M=R_z(-\psi_0)(p_L-p_0),\qquad v_M=R_z(-\psi_0)v_L.
\]

先做传感器到中心的换算，再冻结起点；不是程序启动后任意第一帧。验证样例：`p0=(2,-1),ψ0=90°`，则`(2,-1)→(0,0)、(2,0)→(1,0)、(1,-1)→(0,1)`；速度`(0,1)→(1,0)`，只旋转不减起点。复用`freeze_T_map_odom()`及既有A样例。

### A7. 外参量完怎样验：两层实验

**第一层：光学方向和距离。**按B2摆码、看x/y/z。码像素中心靠近内参主点`(715.85,589.08)`时x/y接近0；它不等于图像几何中心。此实验不能证明全部安装外参。

**第二层：安装变换。**需要独立知道码中心在G系中的`p_G真`，不能拿同一相机待验证的结果当真值。

1. 相机和G保持固定。用G原点/代理点及已知轴建立共同测量基准，用尺/网格/直角尺摆至少6个位置，覆盖近远、左右和不同高度，避免全在一条直线上。
2. A/C独立测每个码中心的三维分量及误差；B每位置录约5秒`p_O`、有效性、重投影误差。
3. 算`p_G算=R_G←C R_C←O p_O+t_G←C`，逐点对照独立测量，记三轴和总误差。
4. 调整初值只用其中一部分位置，留至少2个未参与调整的位置验收。

如果机械基准无法给出足够准确的旋转，但已独立量出这些点在G中的三维坐标，A可在同一组点对上做成熟的三维刚体配准：求使`Σ||p_G真-(R_G←O p_O+t_G←O)||²`最小的R/t，约束R为旋转矩阵。用去中心化点对、SVD和行列式修正即可，不必一开始上完整手眼标定。此时拟合结果已包含光学轴转换，`R_G←C=R_G←O (R_C←O)ᵀ`，不能再多乘一次。只有单张图、G原点/轴未定义或所有点真值仍由同一相机推算时，这条路也不成立。先完成测量/验证，再按残差决定是否需要拟合，不把它额外强加为第一轮任务。

旋转可信、平移存疑时，可检查`t_i=p_G真-R_G←C R_C←O p_O`是否接近常量，再估计平移均值。误差随距离放大，先查旋转、K和码尺寸；恒定偏移先查原点；随方位变化先查旋转/畸变/符号。这是排查线索，不是仅凭现象确定原因。

团队初始排错门槛：1—3m且独立测量足够准确时，位置误差不超过`max(0.03m,距离×3%)`。不是官方标准或最终成绩保证。测量基准本身不准，就标“未验证”，不放宽阈值强行通过。

### A8. 时间和预测也要有明确操作

1. 在记录表分清图像stamp、PnP沿用的图像stamp、IMU stamp、LIO stamp、tick stamp和到达时间。消息stamp是否代表采样/接收时刻要查实际实现，不能自动视为物理同步。
2. B录B2的角点/PnP短bag，A/B核对**同一输入帧**的stamp完全一致。任意各echo一条不要求相同。
3. 写清目标变换应查询图像时刻的底盘/云台姿态；缺时间样本时要有明确无效/外推规则，不无提示换成“最新姿态”。
4. 用样例：`p(t_obs)=(1,2),v=(0.4,-0.2),t_tick=t_obs+0.04s`。当前位置应为`(1.016,1.992)`，100ms后为`(1.056,1.972)`。只加0.1s会漏图像到tick的40ms。
5. A/B对照现有估计与写出函数。当前估计器仍使用到达/单调时间；PnP保留stamp不等于整链已按观测时间对齐。将要改的行为和样例交B。

[A_匀速轨迹预期.csv](../阶段任务一/A_匀速轨迹预期.csv)是已有数学样例；配套[样例说明](../阶段任务一/README.md)。B4源是静止样例，不能互相逐行比。预测目标速度也不是CSV的建议底盘速度。

**A交付：**记录表中坐标/测量/计算页、轴向照片、独立误差对照和给B的明确变换/时间定义。AI可算矩阵和统计；轴、方向、测量依据和合理性由A核对。

## 四、B：从软件准备到可检查的输出

### B0. 阅读和用途

- [环境说明](https://blog.secaumene.com/kit/campus-contest-env.html)：“按tick写CSV”“赛前用哑程序自测”“比赛程序的常见坑”，直接用于B4。
- [协议](https://blog.secaumene.com/kit/protocol.html)：“传输”“0x01 IMU”“0x30角速度指令”“方向约定”，用于B3理解单位、姿态和就绪。
- [第四课](https://blog.secaumene.com/teach/class-4.html)：“延迟定量分析”，区分函数耗时、输入延迟和完整行可见延迟。
- 第二课“PnP”“误差分析”和第一课“TF2”，用于视觉和坐标接入，不重写求解器。

### B1. 版本、目录、构建

Mac本地仓库是`/Users/a1/Documents/RBM/github/rm_task1-repo`。在该根目录记录：

```bash
git rev-parse HEAD
git status --short
git diff --stat
```

代码有未提交修改，提交号不足以证明版本一致。Ubuntu使用实际复制路径，不照抄Mac路径。至少检查以下仓库内路径内容存在且为本次版本：

```text
src/gimbal_tracker_bringup/launch/vision.launch.py
src/gimbal_tracker_bringup/launch/submission.launch.py
src/gimbal_tracker_bringup/config/camera_team47.yaml
src/gimbal_serial_bridge/gimbal_serial_bridge/hardware_adapter_node.py
src/gimbal_serial_bridge/gimbal_serial_bridge/command_gate.py
42-gimbal_serial/launch/gimbal_serial.launch.py
tools/submission_test_source.py
tools/observe_submission.py
```

比赛源目录使用队号前缀；目录名变了，ROS包名仍以`package.xml`为准。同一工作区不能同时放原包和47前缀副本。42串口包也须在工作区src扫描范围内。

先在个人Ubuntu或确认后的选手工作区清点：

```bash
cd ~/ws_championship
source /opt/ros/humble/setup.bash
export ROS_LOCALHOST_ONLY=1
colcon list --base-paths src
python3 -c 'import numpy, cv2; print(cv2.__version__); print(hasattr(cv2, "aruco"))'
```

预期12包各一次：`gimbal_common、gimbal_interfaces、gimbal_perception、gimbal_serial_bridge、gimbal_serial_42、gimbal_tracker_bringup、grid_mapper、gimbal_judge_io、gimbal_localization、gimbal_estimation、gimbal_control、chassis_advisor`。OpenCV有aruco，记录版本。

找不到包先查复制位置；重复包先确认唯一副本；缺依赖先保留具体报错并核对已有环境，不直接跑整套安装脚本或构建导航。清点通过后定向构建：

```bash
colcon build --symlink-install --packages-select gimbal_common gimbal_interfaces gimbal_perception gimbal_serial_bridge gimbal_serial_42 gimbal_localization gimbal_estimation gimbal_control chassis_advisor gimbal_tracker_bringup grid_mapper gimbal_judge_io --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 pkg prefix --share gimbal_tracker_bringup
ros2 pkg executables gimbal_serial_bridge
```

失败先处理第一条实际错误，不继续launch。prefix应指本次工作区；可执行列表应有`hardware_adapter_node`。指向旧工作区就查source顺序。新增launch/YAML未安装时，重构建对应包并重新source。

在含tools的实际仓库根目录，可复核已有离线入口：

```bash
python3 -B tools/verify_submission_offline.py
python3 -B tools/verify_coordinate_regressions.py
python3 -B tools/verify_vision_and_gate.py
```

个人电脑已有隔离依赖时，可用`.venv/bin/python -B tools/verify_all_offline.py`一次复核完整离线检查；结果和逐项日志在`build/offline/`。它不安装依赖，不启动ROS/真实硬件。不要将个人电脑venv、build、install、log、合成CSV推送进Git。NUC不通过pip覆盖ROS的OpenCV/numpy。

### B2. 视觉：先输入，再ID、方向、距离、时间

地点为NUC同域终端；C摆目标，A测和算，B操作程序。保持底盘/云台固定，不启控制器。

**1. 输入核对：**

```bash
ros2 topic info /image_raw --verbose
ros2 topic echo /camera_info --once --field width
ros2 topic echo /camera_info --once --field height
ros2 topic echo /image_raw --once --field width --qos-reliability reliable
ros2 topic echo /image_raw --once --field height --qos-reliability reliable
ros2 topic echo /image_raw --once --field header --qos-reliability reliable
```

预期唯一相机publisher、Reliable，原图width/height为1440/1080、header持续更新。不能只读camera_info就认定原图尺寸正确；原图不符时先停止当前测距验收，查裁剪/缩放与K。不要echo大图全部字节。用已有RViz的Image显示`/image_raw`、Reliability设Reliable；不是再开相机驱动。

全零K使用本队YAML解决，不据此重标定。`topic hz /image_raw`可能因大图订阅丢帧偏低；`camera_info`频率也不能当处理率。

**2. 启动和选择ID：**

```bash
ros2 launch gimbal_tracker_bringup vision.launch.py
```

保持该终端；另一个已source的同域终端执行，3替换成主办方ID：

```bash
ros2 param set /marker_pnp_node perception.target_digit 3
ros2 param set /aruco_detector_node perception.target_digit 3
ros2 param get /marker_pnp_node perception.target_digit
ros2 param get /aruco_detector_node perception.target_digit
ros2 topic echo /perception/marker_cam --once --qos-reliability best_effort
```

默认ID=-1，没有输出是等指定目标的正常状态。两次get都应为3；指定码可见时`marker_id=3、valid=true、frame_id=camera_optical_frame、z>0`，重投影误差不超过当前2px门限。启动日志fx约1778.23、码尺寸0.10m，配置是本队YAML。其它码不应被当成指定码。

参数名`target_digit`是历史命名，仍表示ID。此方式用于赛前一次选择，不声称支持任意时刻无缝换目标。

**3. C摆码、B看符号、A读回解释：**每次固定约5秒。

| C操作 | 预期PnP变化 |
|---|---|
| 正对相机，码中心接近主点(715.85,589.08) | x/y接近0、z为正 |
| 向图像右移 | x增大 |
| 向图像下移 | y增大 |
| 沿光轴移远 | z增大 |
| 遮挡后恢复 | 遮挡后无新有效观测，恢复后stamp更新 |

本批会在目标丢失、旧图像或PnP拒绝时报告无效，并清除上一帧初值。消息仍有心跳不代表有效观测更新；检查valid和图像stamp。符号通过不等于安装外参通过。

**4. 测距：**在光轴附近约1m、2m、3m摆码；空间不足用能量准的三档，写实际值。A量光心到码中心的前向深度；光心只是估计就记误差。偏轴时斜距为`sqrt(x²+y²+z²)`，不能拿它直接对z。

B每档保留5秒消息，至少20条有效输出，再统计均值/标准差/偏差与有效条数；按A7排错门槛评估。可用echo看数值、短bag保留原始数据。画面模糊时先查焦点、曝光、反光和码弯曲，不先调滤波器。

**5. 时间：**另一个同域终端录：

```bash
ros2 bag record -o /tmp/team47_vision_stamps_001 /perception/marker_corners /perception/marker_cam
```

目录须新，5—10秒Ctrl+C。只录角点/PnP，小消息足够验传播时间戳。A/B按同一输入帧核对stamp；不是任意各echo一条。提取方式见第七部分。

| 故障 | 按顺序排查 |
|---|---|
| 无图像/节点都看不到 | 是否NUC、ROS域、source环境、已有驱动 |
| 有publisher却收不到 | `topic info --verbose`的类型、QoS |
| 有清晰码无角点 | 两节点ID、DICT_4X4_50、aruco模块、曝光/码像素尺寸 |
| 有角点无有效PnP | K/D加载、0.10m、角点顺序、拒绝原因 |
| 居中附近重投影异常 | OpenCV版本；当前已比较IPPE、LM精修和ITERATIVE候选，若真实数据仍异常，保留图像与版本定点复查 |
| 距离固定比例偏 | 码黑色外边长、单位、图像缩放与K |
| 边缘误差变大 | 畸变、重复去畸变、分辨率/裁剪与标定是否一致 |

### B3. 云台本轮只接收反馈：关闭桥的指令订阅

前提：C确认真实端口和占用。已有桥时，由原启动者确认身份、实际关闭指令订阅、TF发布与本轮约定一致，再直接订阅；发送行为未确认不能默认“只收反馈”，交原操作者核对，不开第二桥抢设备。

本批`hardware_interface.launch.py`已默认关闭桥的指令订阅及adapter转发。下面仍分两步启动，便于核对端口和每个节点；只收反馈时必须看到`subscribe_cmd_vel=false`。不要把它改成true，也不要启动控制器。

终端1启动，端口替换为C确认值：

```bash
ros2 launch gimbal_serial_42 gimbal_serial.launch.py port:=/dev/gimbal subscribe_cmd_vel:=false publish_tf:=false imu_topic:=/red_standard_robot1/gimbal
```

终端2：

```bash
ros2 run gimbal_serial_bridge hardware_adapter_node --ros-args -p control_enabled:=false
```

终端3核对：

```bash
ros2 param get /robo_bridge subscribe_cmd_vel
ros2 param get /robo_bridge publish_tf
ros2 param get /robo_bridge imu_topic
ros2 param get /hardware_adapter_node control_enabled
ros2 topic info /red_standard_robot1/gimbal --verbose
ros2 topic echo /red_standard_robot1/gimbal --once --qos-reliability reliable
ros2 topic echo /gimbal/state --once --qos-reliability best_effort
ros2 topic echo /task1/gimbal_reset_epoch --once --qos-reliability reliable --qos-durability transient_local
```

前四项预期false、false、正确IMU话题、false。IMU唯一publisher，frame为`gimbal_link`。上电要等到温就绪，以首帧有效IMU为准，15秒提示不自动等于损坏。

静止检查：四元数有限、模接近1；gyro接近0并记录偏置；比力模约9.8m/s²。倾斜时不要求只有z非零。adapter中`enabled=false`、关节速度NaN表示机械状态未知，不因此判断故障。

失败顺序：端口/权限→占用→连接/电源状态→到温→协议/波特率。不要用chmod777、杀不明进程、反复抢串口替代定位。本手册不提供使能、点动或重启电源命令。

### B4. 提交实验：20拍通过，再做1800拍

目的只为验按tick写CSV/地图，个人Ubuntu即可；不评定位/方向精度，不需要硬件。

另开三个已source的终端，进入含tools的实际仓库根目录，均设置：

```bash
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=87
```

先确认87未被占用。该域不混跑真实算法；合成源也发内部目标、底盘、建议、地图话题。只在`/test/tick、/test/replay_start`发合成评分消息。

**20拍冒烟，目录必须全新，已有非空就换编号：**

```bash
# 终端1，先运行并保持
ros2 launch gimbal_tracker_bringup submission.launch.py team_id:=47 output_dir:=/tmp/rm_task1_smoke_001 tick_topic:=/test/tick replay_topic:=/test/replay_start
```

```bash
# 终端2，先于源运行
python3 -B tools/observe_submission.py /tmp/rm_task1_smoke_001/47.csv --count 20 --timeout 60
```

```bash
# 终端3，最后运行
python3 -B tools/submission_test_source.py --count 20
```

源等待必要订阅、预热2秒、发20拍、收尾。终端3完成且终端2打印JSON后，再在终端1 Ctrl+C。保存这轮结果。

**1800拍，换全新目录：**

```bash
# 终端1
ros2 launch gimbal_tracker_bringup submission.launch.py team_id:=47 output_dir:=/tmp/rm_task1_stage1_001 tick_topic:=/test/tick replay_topic:=/test/replay_start
```

```bash
# 终端2，先于源运行
python3 -B tools/observe_submission.py /tmp/rm_task1_stage1_001/47.csv --count 1800 --timeout 240
```

```bash
# 终端3
python3 -B tools/submission_test_source.py --count 1800
```

约预热2秒＋180秒＋收尾1.2秒，等待订阅最多30秒。未完成不关闭终端1。结束后：

```bash
wc -l /tmp/rm_task1_stage1_001/47.csv
head -n 1 /tmp/rm_task1_stage1_001/47.csv
tail -n 1 /tmp/rm_task1_stage1_001/47.csv
cat /tmp/rm_task1_stage1_001/47_map.yaml
```

CSV无表头、九列，前八列每行数值应为`1,2,1,2,0.5,-0.25,0.3,0.4`；最后列为本轮tick stamp，六位小数、递增。预测=当前，因为目标静止；建议速度不等于目标速度。

观察器：`passed=true、rows=1800、errors=[]、partial_tail_bytes=0、over_60ms=0`；最大完整行可见延迟<60ms。官方迟到线80ms。源结束后`wc -l`须正好1800，补查计满后是否多行。观察器只认这个固定样例，不能拿来验证正式比赛CSV或A的匀速轨迹。

地图应有47_map.pgm/yaml，4×3格、0.5m/cell、origin[-1,-0.5,0]。PGM从顶行向下灰度：

```text
254 254   0 254
254   0 254 254
  0 254 254 205
```

这用于查上下翻转/未知格，不是正式地图。已有map_server时的加载步骤见第七部分。

| 失败现象 | 下一步 |
|---|---|
| 等订阅30秒报错 | 三终端域/source、输出节点、话题映射/QoS |
| 文件非空 | 换新编号，不截断旧CSV |
| 前八列不对 | 混入真算法、用错样例、旧入口速度守卫改值 |
| 少行/格式错 | tick订阅、写异常、九列设置、flush；不事后补拍 |
| 正确但超时 | CPU/写盘/回调阻塞，保留max和p99，不只看平均 |
| 无图/倒置 | 地图订阅/保存时机、YAML引用、ROS到PGM行序 |

**B交付：**版本/改动、构建日志、视觉/反馈证据、20及1800拍原始产物。每项标通过/失败/未执行；报错带命令和第一条实际错误上下文，不只说“启动不了”。

## 五、C：把设备、真实输入和定位状态查清楚

### C0. 阅读和用途

| 资料 | 章节 | 现场用途 |
|---|---|---|
| [导航入门](https://blog.secaumene.com/nav-intro/) | “3.1为什么要建TF树”“3.2推荐的TF树结构”“3.4新手必踩的三个坑” | 参考点、坐标链、每条TF唯一发布者 |
| [MID360](https://blog.secaumene.com/kit/mid360-lidar.html) | “两份配置文件”“话题与xfer_format”“Point-LIO的要求”“IMU的单位是g” | 输入类型、配置、单位核对 |
| [定位教程](https://blog.secaumene.com/nav-intro/localization-planning-control.html) | “5.1里程计：point_lio” | 里程计输出与底盘中心的区别 |
| [导航实操](https://blog.secaumene.com/nav-intro/practice.html) | 工程启动、建图/点云保存相关部分 | 对照实际输入，不先启动完整导航 |
| [协议](https://blog.secaumene.com/kit/protocol.html) | “传输”“0x01 IMU”“方向约定” | 串口、就绪、反馈单位 |

### C1. 安装和端口

1. 拍最终装配整体、俯视、侧视、IMU/相机固定点和雷达安装；照片编号写入记录表。
2. 与A做A2—A5。C量，A读回起点/终点/谁的轴/正负/单位，再确认记录。
3. 写零位的可复现识别方式、允许范围和停止方式。没有标记就问清参考方法，不能只写“角度0”。
4. 查设备别名：

```bash
ls -l /dev/serial/by-id
ls -l /dev/gimbal
```

路径不存在只说明无此别名。结合设备序列号/管理员说明确认，不能选第一个ttyUSB。已确认后可用`fuser -v /dev/实际设备名`只查占用，不杀不明进程。把真实设备路径交B。

交接表须连上“设备/序列号→路径→占用程序→ROS节点/话题”。不读取保护目录查配置，由实际驱动操作者提供允许范围内的信息。

### C2. 雷达和IMU：类型、QoS、单位、更新都要查

相机与B2共做一次，不重复启驱动。雷达在NUC同域执行：

```bash
ros2 topic info /red_standard_robot1/livox/lidar --verbose
ros2 topic type /red_standard_robot1/livox/lidar
ros2 topic type /red_standard_robot1/livox/imu
ros2 topic echo /red_standard_robot1/livox/imu --once
ros2 topic hz /red_standard_robot1/livox/lidar
```

频率观察约10秒Ctrl+C，记录实际值。若只有`/livox/lidar、/livox/imu`，对这两个已知话题重复检查并记录，不假定两组名字类型/来源相同。记publisher、frame_id、QoS、stamp、频率；不反复打印大点云。

Point-LIO Livox分支要求`livox_ros_driver2/msg/CustomMsg`及`sensor_msgs/msg/Imu`。历史记录中的PointCloud2不满足前者。

| xfer_format | 驱动输出 | 判断 |
|---|---|---|
| 0 | lidar为PointCloud2 | 需由有权限的驱动操作者调整已有配置，不启动LIO假装接通 |
| 1 | lidar为CustomMsg | 核对命名空间和IMU后可进入C3 |
| 4 | lidar为CustomMsg，lidar/pointcloud为PointCloud2 | LIO用前者，RViz可用后者 |

加`/red_standard_robot1`命名空间后规则相同。只改名不能转换类型；不另启第二个驱动，不随意改IP或拔线。

静止Livox IMU加速度模约1，单位g；云台IMU约9.8m/s²，两者不同。Point-LIO的`mapping.acc_norm=1.0`对应g。记录三分量和模长，倾斜时不要求z恰为1。

### C3. Point-LIO：输入齐全才单独启动

前提：选手环境已有包、无重复实例、CustomMsg/IMU持续更新、来源匹配、版本/配置确认。导航代码目前在`archives/robomaster_navigation.tar.gz`，未展开到本仓库，不能认为复制core就已安装导航。

```bash
ros2 pkg prefix --share point_lio
ros2 node list
```

包缺失交B记录实际依赖/构建缺项，不直接全量装导航。包已安装且与审查源码一致，输入在规则命名空间下时：

```bash
ros2 launch point_lio point_lio.launch.py namespace:=red_standard_robot1 rviz:=false
```

原默认mid360.yaml用相对`livox/lidar、livox/imu`；实际版本不同交B定向适配，不猜命令。另一同域终端：

```bash
ros2 topic info /red_standard_robot1/aft_mapped_to_init --verbose
ros2 topic echo /red_standard_robot1/aft_mapped_to_init --once
ros2 topic hz /red_standard_robot1/aft_mapped_to_init
```

持续有限位姿只说明LIO开始工作。记`frame_id、child_frame_id`、算法跟踪参考点和stamp语义。此launch的TF在namespace下，还要查`/red_standard_robot1/tf、tf_static`；不能因全局/tf没有新边判没输出。

本次压缩包源码`point_lio/src/laserMapping.cpp:publish_odometry()`中，Odometry为`camera_init / body`，TF边却名为`camera_init → aft_mapped`，不能只按名字猜参考点。若现场版本相同，可查询：

```bash
ros2 topic info /red_standard_robot1/tf --verbose
ros2 run tf2_ros tf2_echo camera_init aft_mapped --ros-args -r /tf:=/red_standard_robot1/tf -r /tf_static:=/red_standard_robot1/tf_static
```

看数值更新约10秒Ctrl+C；无结果先核对实际版本、frame名、`publish.tf_send_en`和重映射，不另造第二条同名TF。原桥当前用主机接收时刻给云台IMU打stamp（`bridge_node.cpp:onFrame()`中的`now()`）；原LIO默认用`lidar_end_time`，二者不同，A6静止对齐不等于时延标定。

静止30秒记起止位置/姿态和漂移；再由管理员允许的短直行、转向、回标记点实验取得实测位移/返回偏差。C不发送底盘控制。看着稳不等于精度合格。

若LIO输出为`T_L←S`，S是算法实际雷达/IMU参考点，需要安装关系`T_B←S`：

\[
T_{L\leftarrow B}=T_{L\leftarrow S}(T_{B\leftarrow S})^{-1}.
\]

先确认S，别永远当作雷达壳中心。本批base_pose_provider已实现这个换算：`localization.sensor_in_base_xyz/rpy`填写T_B←S，`sensor_extrinsics_confirmed=true`只能在实测确认后填写。输入父系不叫odom时，还需T_odom←L及`parent_transform_confirmed=true`。不能靠改话题名修正参考点偏差。本版本Point-LIO未填twist，用`odom_twist_source=pose_difference`按源stamp差分；不是将零twist认作静止。

如A6的前提齐全，静止同录：

```bash
ros2 bag record -o /tmp/team47_alignment_001 /red_standard_robot1/gimbal /red_standard_robot1/aft_mapped_to_init
```

录5—10秒Ctrl+C，A/C记姿态与零位。LIO没运行就只录IMU，明确缺另一条流；不能拿两个不同实验的任意消息对齐。

### C4. 地图：先确定真实来源，再保存

Point-LIO有点云/里程计，不代表有二维占用地图。当前`mapping.launch.py`默认把field.yaml栅格化，不是雷达现场建图。

1. 查**选手自己的**/map或实际命名空间话题，类型应为`nav_msgs/msg/OccupancyGrid`，确认生产节点和数据来源，不取评分真值。
2. 有真实地图，检查metadata、frame、分辨率、原点，由B用topic模式接保存；下面/map换实际自有地图名，目录全新：

```bash
ros2 launch gimbal_tracker_bringup mapping.launch.py grid_map_source:=topic grid_map_topic:=/map maps_dir:=/tmp/team47_realmap_001
```

此命令消费地图，不生产雷达SLAM。没输入时等待不算成功。

3. 没有OccupancyGrid，就交LIO点云/位姿/TF证据，由B选择并接最小建图子链。这是尚待完成的真实接入项，不能拿field.yaml示例顶替。
4. 不直接开`rm_navigation_reality_launch.py`或完整bringup来解地图：原入口还包含驱动、Nav2、joy/控制，不是单独建图命令。
5. 地图出现后，在RViz用其真实frame显示，核对实际墙角/通道、占用/自由/未知区，再查PGM方向/YAML原点。地图系与比赛起点不一致时，要实际变换几何，不能只改header名字。

最终由提交链保存为`~/ws_championship/output/47_map.pgm、47_map.yaml`。任意测试目录有PGM不代表完成比赛地图。

**C交付：**安装/设备/端口表、原始测量和照片、零位/停止方式、输入类型/单位/QoS、静止与LIO记录、地图来源或明确缺项。AI可整理日志，实物连接和实验状态由C亲自核对。

## 六、汇合后怎么推进，不跳过缺口

### 6.1 坐标链的三个验证场景

A给变换和不确定项，B接实际安装与观测时刻，C提供独立测量和装置状态：

1. 全部静止：目标比赛坐标应稳定，与独立测量相符。
2. 底盘固定、目标固定、云台改变姿态：图像位置会变，目标比赛坐标应基本不变；绕圈先查外参、杠杆臂、世界对齐/时间。
3. 目标固定、管理员移动底盘：底盘位置随实测变化，目标比赛坐标基本不变；跟车走先查底盘变换，随速度变差查时间。

第2/3项涉及运动，仅在下列控制/现场条件齐全后按管理员允许流程做。本手册不给运动指令。本批已有历史姿态插值与图像时刻TF查询；现场还需确认TF足够新、链路唯一，且A的实测值实际进入配置。

### 6.2 预测和方向建议怎样接

先保留已有估计器/方向算法，按数据修必要部分，不新建三套系统：

- 用A8样例落实`cur(t_tick)`、`pred(t_tick+0.1)`，不是任意“最新发布时刻+0.1”。
- 先静止、匀速样例，再用允许采集的运动记录核验100ms后实际位置；没真实参考时只能说模型计算通过。
- 建议速度和底盘/目标/地图同属比赛系。无遮挡正前目标应向前；有障碍时检查建议方向是否穿过不可通行区。建议仅送评分，不控制底盘。
- 现环境规则：建议速度模<0.02m/s按静止，目标距离<0.5m为零误差，否则最大误差；不能用全零掩盖缺失功能，临赛与最终版再核对。

### 6.3 yaw/pitch控制前需要哪几项证据

| 前提 | 交付人和证据 |
|---|---|
| 图像方向/安装 | A/C的变换表及B2位置/符号结果 |
| IMU与LIO对齐 | A6和另一姿态核验；MCU复位后重新确认 |
| 关节零位、限位、停止 | C的可复现说明，不用world yaw冒充机械关节角 |
| 控制器适配 | 离线已修惯性姿态/关节限位混用；仍需据实测装配选择适用控制入口，默认关闭控制/搜索/前馈 |
| 超时/复位门控 | B对应逻辑结果，再按许可现场验证，离线通过不冒充下位机实测 |

齐全后，B再提供一份写明允许角速度、持续时间、停止条件和现场观察人的控制试验步骤。当前不丢一条enable命令让大家试转：机械信息和控制修正确实尚缺。这是有负责人和前置条件的后续工作。

## 七、记录如何变成统计与验收结果

### 7.1 短bag怎样读，不只交一堆文件

在有Humble和本队消息包的Ubuntu终端，先用`ros2 bag info /tmp/team47_vision_stamps_001`核对录到的话题、条数、起止时间。

需要看消息可在**独立回放域**（例如88，确认无人使用）开两个已source的终端，均设置`ROS_LOCALHOST_ONLY=1、ROS_DOMAIN_ID=88`，不启动任何控制/实机桥：

```bash
# 终端1：先监听PnP，文本保存到本次记录目录
ros2 topic echo /perception/marker_cam --qos-reliability best_effort > /tmp/team47_vision_stamps_001_pnp.txt
```

```bash
# 终端2：播放记录；路径换成本次实际bag
ros2 bag play /tmp/team47_vision_stamps_001
```

播放完成再Ctrl+C终端1。查角点可重复回放并改成marker_corners；用同一stamp对应，不要求两个文件第n条一定配对，因为有拒绝帧。记录解析/均值统计可借助AI，B负责核对字段和过滤条件，A负责比较独立实测。原始bag保留。

上面重定向路径也应换新编号，防止覆盖旧文本。不要在驱动/控制域直接回放IMU或合成状态；旧时间戳本来就可能被新鲜度门控拒绝，不能据此判断实时链坏了。

### 7.2 固定位置的数据怎样算均值、波动和stamp配对

一个固定码位置单独录一份B2的bag，至少5秒；移动目标时不要把多个位置混成同一份统计。下面在已source的Ubuntu终端读bag，不回放、不发任何消息。只读取该bag的角点/PnP，要求已有Humble的rosbag2_py和本队接口；缺模块时先记录环境缺项，不临时用pip装ROS包。

把命令最后的路径替换为这一固定位置的实际bag路径：

```bash
python3 - /tmp/team47_vision_stamps_001 <<'PY'
import json, math, statistics, sys
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

reader = rosbag2_py.SequentialReader()
reader.open(rosbag2_py.StorageOptions(uri=sys.argv[1], storage_id='sqlite3'),
            rosbag2_py.ConverterOptions('', ''))
types = {t.name: t.type for t in reader.get_all_topics_and_types()}
corners_stamps, obs_stamps, points, reproj = set(), [], [], []
def stamp(m):
    return m.header.stamp.sec * 1000000000 + m.header.stamp.nanosec
while reader.has_next():
    topic, data, recorded_time = reader.read_next()
    if topic not in ('/perception/marker_corners', '/perception/marker_cam'):
        continue
    m = deserialize_message(data, get_message(types[topic]))
    if topic.endswith('marker_corners'):
        if len(m.polygon.points) == 4:
            corners_stamps.add(stamp(m))
    elif m.valid and all(math.isfinite(v) for v in (m.x,m.y,m.z,m.reproj_err_px)):
        obs_stamps.append(stamp(m))
        points.append((m.x,m.y,m.z))
        reproj.append(m.reproj_err_px)
if not points:
    raise SystemExit('没有有效PnP，先查话题、ID、输入和拒绝原因')
axes = list(zip(*points))
print(json.dumps({
    'valid_count': len(points),
    'mean_optical_xyz_m': [statistics.mean(a) for a in axes],
    'std_optical_xyz_m': [statistics.pstdev(a) for a in axes],
    'max_reprojection_px': max(reproj),
    'observations_without_matching_corners_stamp': sum(s not in corners_stamps for s in obs_stamps),
}, ensure_ascii=False, indent=2))
PY
```

适用默认SQLite格式的Humble bag；若实际存储不同，先用bag info确认，不盲目把storage_id换成猜测值。有效数至少20；均值/标准差填记录表。匹配缺项为0是预期；非零先查bag是否完整、录制QoS是否丢消息，再查stamp传播，不能只凭录制缺帧认定节点错。重投影小不能单独证明K、尺度和外参正确。

### 7.3 合成地图怎样真正加载

先确认已有包：`ros2 pkg prefix nav2_map_server`。不存在就记录“map_server实际加载未验证”，不为此启动/安装完整Nav2。

存在时，合成源已停止、仍在隔离域87，用终端1：

```bash
ros2 run nav2_map_server map_server --ros-args -r __node:=stage1_map_server -p yaml_filename:=/tmp/rm_task1_stage1_001/47_map.yaml
```

终端2同环境/域：

```bash
ros2 lifecycle set /stage1_map_server configure
ros2 lifecycle set /stage1_map_server activate
ros2 topic echo /map --once --field info --qos-reliability reliable --qos-durability transient_local
```

两次状态转换成功，info为4×3、0.5m、正确原点。可在同域RViz将Fixed Frame设map，Add Map选择/map，Durability为Transient Local，核对非对称占用和未知格。结束Ctrl+C map_server，关闭隔离域终端。

### 7.4 总验收：逐项记实际状态

| 项目 | 通过证据 | 不足时怎样记 |
|---|---|---|
| 实物坐标 | 原点/轴有图有依据，固定/运动关系明确 | 缺具体哪个轴/原点/零位 |
| 外参 | 原始mm、旋转依据、误差和独立位置对照 | 机械初值/光心未知/未验证分开写 |
| 软件 | 版本含改动、构建结果、安装前缀、日志 | 静态检查不算Humble通过 |
| 视觉 | ID、K/D、方向、距离、遮挡、同帧stamp | 各项失败数据与原因 |
| 反馈 | 唯一接收桥、参数、姿态/单位/复位语义 | 无数据/未就绪/延迟未测分开 |
| 提交 | 20/1800拍原始产物、逐拍九列/六位时间、延迟、地图内容 | 不补写、不把合成通过算精度通过 |
| LIO | 正确类型/单位，持续自有位姿，跟踪参考点明确 | 未安装/未启动/漂移未测分开 |
| 地图 | 实场来源、障碍/坐标/保存和加载 | 示例图/只有点云不算完成 |

离开前，每项要么有结果，要么有具体缺项、负责人和下一步。“继续看坐标”“再调算法”不算交接。

## 八、源码在哪里，谁改，怎样避免冲突

本地完整根目录：`/Users/a1/Documents/RBM/github/rm_task1-repo`。下表为根目录内精确路径；在NUC用`colcon list --base-paths src`找实际带队号目录，不使用省略号。

| 行为 | 路径/函数 |
|---|---|
| 起点/旋转 | src/gimbal_common/gimbal_common/frames.py：rpy_to_mat、freeze_T_map_odom |
| 光学观测换算 | src/gimbal_estimation/gimbal_estimation/observation_builder.py：ObservationBuilder |
| 估计时刻 | src/gimbal_estimation/gimbal_estimation/target_estimator_node.py |
| 世界/起点关系 | src/gimbal_localization/gimbal_localization/map_frame_broadcaster_node.py |
| 底盘来源 | src/gimbal_localization/gimbal_localization/base_pose_provider_node.py |
| 检测/ID | src/gimbal_perception/gimbal_perception/aruco_detector_node.py：_on_timer |
| PnP/沿用stamp | src/gimbal_perception/gimbal_perception/marker_pnp_node.py：_on_corners、_publish |
| PnP拒绝 | src/gimbal_perception/gimbal_perception/pnp_solver.py：solve |
| 串口 | 42-gimbal_serial/launch/gimbal_serial.launch.py、42-gimbal_serial/src/bridge_node.cpp |
| 适配/门控 | src/gimbal_serial_bridge/gimbal_serial_bridge/hardware_adapter_node.py、command_gate.py |
| 提交 | src/gimbal_judge_io/gimbal_judge_io/judge_io_node.py、tick_csv.py |
| 地图消费/保存 | src/grid_mapper/grid_mapper/grid_mapper_node.py、pgm_io.py |

A/C提交测量和数学定义，B负责这轮核心接入文件；不三个人同时改tracker_params.yaml。相机外参由哪份配置、哪一个TF发布者使用，B接入时明确唯一来源。只有K/D的camera_team47.yaml不能传给旧static_tf入口当外参。

TF每条边只有一个发布者；不同时起教学描述、旧桥、新定位链。`tf2_echo`有数字只证明查到一套已发布值，不证明测量正确。本批GimbalState注释已更正：quat是R_world←gimbal；yaw/pitch是惯性角，不是机械关节角，未知关节速度为NaN。`/gimbal/joint_states`是同stamp底盘+IMU和实测模型派生的估计，不是编码器实测。

**立即开始仍是：B在个人Ubuntu做20拍提交；A/C确认已固定相机/云台的原点、轴和静态PnP。**交回数据后，再解决真实时间/几何误差；不要在缺数据时继续盲调参数。


## 九、整车固定后如何启用新数据处理链（现在先不执行）

### 9.1 每个配置量对应什么实物

本批新增`src/gimbal_tracker_bringup/config/vehicle_template.yaml`。它是待填模板，确认项全false、epoch为-1。`processing.launch.py`会拒绝原样模板；禁止为消除报错把确认项全改true。

| 配置 | 含义与填写依据 |
|---|---|
| localization.odom_topic / odom_frame_id / odom_child_frame_id | 从C3实际消息读取；本压缩包为aft_mapped_to_init，父camera_init、子body |
| localization.sensor_in_base_xyz/rpy | T_base←Point-LIO的IMU body；A5实测；不是把Livox内部激光/IMU外参再抄一次 |
| localization.source_parent_in_output_xyz/rpy | T_odom←camera_init；确认为同一参考可显式填恒等，不能未经确认只改frame名字 |
| kinematics.base_mount_xyz/rpy | yaw轴安装基准在底盘中的位姿 |
| kinematics.yaw_to_pitch_xyz | yaw转后坐标中，yaw基准到pitch轴基准的位移 |
| kinematics.pitch_gimbal_xyz/rpy | pitch轴后坐标到固件gimbal_link参考点的固定位姿 |
| kinematics.gimbal_camera_xyz/rpy | A2—A4测得的T_G←C；C到光学O的固定轴换算由程序提供，勿重复乘 |
| kinematics.joint_zero_rad / limits_rad | 实测零位偏置和单圈限位分支，单位rad；不是IMU初始yaw |
| kinematics.odom_world_rpy | A6求得R_odom←IMUworld；不要与逆方向world_odom参数混用 |
| kinematics.calibrated_epoch | 对齐时同一次桥会话的`/task1/gimbal_reset_epoch`值；重启桥或MCU后重新确认 |
| grid.map_topic / source_frame | 自有真实OccupancyGrid和其实际frame；不是点云，也不是field.yaml示例 |

模型顺序为T_B←mount · Rz(yaw+zero) · Trans(yaw_to_pitch) · Ry(pitch+zero) · T_pitch←G · T_G←C · R_C←O。rpy均为固定轴XYZ，即Rz(yaw)Ry(pitch)Rx(roll)。A用非零角和非零偏移做独立核算，不能只验全零姿态。四元数/参数/照片统一记录方向和单位。

### 9.2 Point-LIO单独入口

原包已安装、C2输入验证通过且没有另一份LIO实例时，可以使用本批只启动算法的入口：

```bash
ros2 launch gimbal_tracker_bringup lio_only.launch.py lidar_topic:=/livox/lidar imu_topic:=/livox/imu
```

若有效CustomMsg/同设备IMU在`/red_standard_robot1/livox/`，替换这两个绝对话题。不要两个都订阅/启动。复用原mid360配置，仅显式输入名，不带驱动、RViz、Nav2和控制；acc_norm=1.0的前提是C2证据表明单位g。输出默认仍为`/red_standard_robot1/aft_mapped_to_init、cloud_registered`。按C3检查实际frame和漂移。

### 9.3 地图选择与TF父链

最短候选是Point-LIO的`cloud_registered`→现有`pointcloud_to_laserscan`→`slam_toolbox`，不必为地图启动完整Nav2。它仍需要正确的高度裁剪、IMU到车体的外参、时间与TF；C先提供LIO云/位姿/实际frame和安装高度证据，再决定参数。旧导航里对terrain代价的intensity=0.1..2.0不能照搬到原始Livox反射强度。没有经过真实数据验证，不能宣布这条链已验收。

建图时，SLAM可发布`lio_map→odom`，底盘提供者发布`odom→base_link`，另有确认的`odom→camera_init`。**这时不启动比赛map_frame节点**，否则odom会同时有两个父节点。保存地图时记录最终T_lio_map←odom；停止SLAM后，若LIO一直保持同一参考系，可将逆变换作为`odom→lio_map`。比赛阶段再发`map→odom`，grid_mapper通过完整链把地图origin变到比赛系。不能只改header。重启LIO会改变参考，旧变换失效，需要重新建图或真实重定位。连续SLAM与比赛原点同跑仍需另一份明确父链设计，当前入口不伪装已经解决。

### 9.4 数据处理启动与验收

所有实测值、唯一TF发布者及地图来源确认后，复制模板为选手自己的绝对路径配置，再启动：

```bash
ros2 launch gimbal_tracker_bringup processing.launch.py vehicle_config:=/实际绝对路径/vehicle47.yaml target_id:=-1 submission:=false
```

此入口不启动驱动、串口桥或云台控制器，只处理现有数据。先检查：`/gimbal/base_pose_raw`保留源stamp且是odom/base_link；回放起点前不伪造map原点；`/gimbal/joint_states`的stamp对应IMU且角度/限位正确；完整TF只有一套。收到真实replay_start且历史底盘位姿覆盖该时刻后，才发比赛系结果。未覆盖、超时、MCU epoch变化都要解决原因，不改成单位阵或最新位置兜底。

```bash
ros2 run tf2_ros tf2_echo map camera_optical_frame
ros2 topic echo /estimation/target_state --once --qos-reliability best_effort
ros2 topic echo /chassis/advice --once --qos-reliability best_effort
```

`tf2_echo`只看当前TF，不证明程序历史查询和真实标定正确；仍做6.1三种几何实验，并保留图像/IMU/自有底盘/算法输出stamp配对。等待主办方回放时不向`/judge/`发布测试信号。需要合成启动信号的实验放个人Ubuntu隔离域，用`/test/`并明确重映射。

数据链验收后停止本处理实例，正式准备阶段再以同配置`submission:=true output_dir:=~/ws_championship/output team_id:=47`启动唯一提交节点。不要同时运行submission.launch和含submission的processing；同一CSV会被单写者锁拒绝。新轮正式文件清理由主办方负责，程序绝不改名/修补已经提交的行。

本入口没有自动瞄准输出，不能因此宣布比赛完成。现有world控制器要求平面底盘/同向安装等条件，纯BearingControlLaw的离线试验也不证明真实装配适用。A/C给出真实关节、轴向和停止方式后，再据数据确定控制接入及限速短时闭环试验。当前control.enabled、search与桥指令订阅均保持关闭。

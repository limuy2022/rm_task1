# 发给B什么，GitHub提交什么

2026-10-17，本次交付说明。**代码与文档一起提交。B收到资料包后，本次向NUC只传一个py。**

## 一、现在发给B

发`47队_发给B_本轮资料_20261017.zip`整个文件，不用逐个挑选。解压后先看`先看这里.md`。

| 包内文件 | 用途 |
|---|---|
| `现场操作/G坐标定义核查_实际操作.md` | B从第4节开始，按命令、预期输出和失败分支操作 |
| `现场操作/watch_gimbal_axes.py` | 本次唯一上传NUC的程序，只订阅已有IMU |
| `理解原理/云台如何转向目标_源码与操作教程.md` | 解释识别→PD→串口→固件，第10节可只读核查现有运行链 |
| `现场结果记录模板.md` | 记录实际结果、失败信息及对应照片/动作 |
| `源码参考/`、`验证结果/` | 教程引用的阅读快照；不需要运行或复制到NUC |

**B的操作顺序：**

1. 在Windows解压，不必切换自己的Linux。
2. 打开G教程，先读第4节；在Windows NoMachine中连接NUC。
3. 按第4.1节创建本队记录目录，只上传`watch_gimbal_axes.py`到打印出的路径。
4. 静止读8秒。有连续数据才按第5节配合A/C核验；没数据走第4.3节，保存报错，不硬判轴。
5. 如需核查PD有没有在跑，照转向教程第10节执行查询；不启动控制器、不改放行参数、不发角速度。
6. 按G教程第8节生成并下载`team47_g_axes_时间.tar.gz`，连同填写的记录模板、必要照片/视频发回A。

无需重新做已经通过的O/C与首次视觉；无需把整份资料包解压进NUC。资料包也不包含可直接试转的电机入口。

## 二、GitHub本轮提交什么

目标仓库：`limuy2022/rm_task1`，本地位置：

```text
/Users/a1/Documents/RBM/github/rm_task1-repo
```

本次交付将最新增量准备为该仓库的未提交修改，供GitHub Desktop核对。完成准备后的文件应为**30个**；本机未代替你Commit或Push。完整清单见交付目录中的`补丁清单.json`。

| 范围 | 文件数 | 包含什么 |
|---|---:|---|
| `42-gimbal_serial/` | 3 | 串口接收/协议拒绝诊断 |
| `src/` | 3 | 高度速度修复、PnP日志节流、另一组内参的参考配置 |
| `tools/` | 13 | G只读工具、标定/检查工具、相应离线回归 |
| `README.md`与`docs/` | 11 | 当前任务入口、G/转向教程、现场/标定说明、验证记录、本交接说明 |

**这30个文件一起提交；不要只提交md。** 控制开关、增益、IMU轴sign没有因本轮修复被打开或改动；另一组内参只作为参考，未替换默认内参。

不向仓库再放：`outputs/`整目录、B资料zip、增量patch、旧上机tar、原始工程tar、历史文档副本、`.venv/`、`build/`、`install/`、`log/`、现场bag/CSV。已有`.gitignore`保留这些排除项。包内源码阅读快照也不再复制一遍进仓库。

## 三、你在Mac上的GitHub Desktop，一步一步提交

1. 打开GitHub Desktop，左上角选对应`rm_task1`仓库，分支应为`main`。如仓库名显示不同，点`Repository → Show in Finder`，确认打开的是上面的`rm_task1-repo`目录。
2. 点左侧`Changes`，应看到本轮**30个文件**。其中应同时出现`src/gimbal_control/.../gimbal_controller_node.py`、`42-gimbal_serial/...`、`tools/watch_gimbal_axes.py`和新教程。若仍是`No local changes`，先核对是否选错本地仓库；不要重新上传zip或拖入整个开发副本。
3. 在确认只有本轮这30个文件后，保持它们全部勾选。绿色`+`是新增、黄色标记是修改；这些是正常源码路径，不是压缩包里的重复副本。
4. 左下角`Summary`复制：

```text
补充云台诊断与教程，修复高度速度估计
```

`Description`可复制：

```text
补充G坐标只读工具、串口接收诊断、现场操作及控制链教程。
修复目标高度低通滞后导致竖直速度偏大的问题，并加入相关回归。
控制18项、适配/门控13项、既有控制单元39项离线检查通过。
保留默认控制关闭；真实固件、IMU方向和实机闭环仍待验证。
```

5. 点击`Commit 30 files to main`（按钮文字可能略有差别）。成功后`Changes`清空，表示已经保存到本地Git历史。
6. 再点顶部`Push origin`。等推送成功，B才能从GitHub拿到。**Commit是本地保存，Push才是上传。** 参见[GitHub Desktop官方说明：提交并推送](https://docs.github.com/en/desktop/making-changes-in-a-branch/committing-and-reviewing-changes-to-your-project-in-github-desktop#write-a-commit-message-and-push-your-changes)。
7. 打开[私有仓库](https://github.com/limuy2022/rm_task1)，查看最新提交标题，并检查`docs/当前任务/云台如何转向目标_源码与操作教程.md`存在。

若提示权限不足，用有该私有仓库协作权限的账号登录；不把仓库改公开。若提示先Pull、冲突或分支保护，保留提示，不点Discard/Force push覆盖队友工作。文件数与这里不符时，先核对变化内容，不能为了凑30把别人的修改取消或删除。

## 四、推送后B怎样拿到源码

- B已有这个仓库：在他自己的GitHub Desktop选仓库，点`Fetch origin`；显示`Pull origin`时点击拉取。
- B还没有本地仓库：登录被邀请的GitHub账号，`File → Clone repository → URL`填`https://github.com/limuy2022/rm_task1.git`，选择Windows本地目录。
- 本轮在Windows下载源码只为阅读/同步；G工具仍在NoMachine里的NUC Ubuntu终端运行。
- GitHub更新不会自动把代码部署到NUC，也不需要为G核查重新构建完整工程。

## 五、当前验证和交付边界

此前的14组全链离线结果保留原范围。本轮控制追踪后实际通过18项控制回归、13项适配/门控回归及39项既有控制单元测试；没有因此宣布ROS、真实固件或电机闭环通过。资料整理阶段只核对文件、链接、zip与补丁，不重复运行算法测试。

本次Git更新的实际准备状态以交付目录`当前使用/代码更新/使用说明.md`与`补丁清单.json`为准。增量补丁保留用于追溯；本地仓库已准备成功后，无需再次应用。

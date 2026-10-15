# gimbal-edu-protocol

云台教学套件的通信协议和接口契约。固件、bridge、虚拟云台和学员答案四方的实现都以这里为准，冲突时改实现，不改契约。

## 里面有什么

- PROTOCOL.md：帧格式，0x30 角速度指令，0x01 IMU 回传，ROS 侧契约，安全状态机，首次上电流程。
- func_id_table.md：已占用的功能号。
- golden_frames.txt：黄金向量，固件、bridge、虚拟云台和课 4、课 5 答案的测试都读它。
- gen_golden.py：生成并自检黄金向量的脚本。

版本以 PROTOCOL.md 开头写的为准（当前 v1.4），其余文件里出现的版本号都是当时的说法。

## 重新生成和校验

只需要 Python 3，没有第三方依赖。生成的结果不含随机数和时间戳，两次生成逐字节相同。

```bash
# 生成 golden_frames.txt 并自检（写到脚本所在目录）
python3 gen_golden.py

# 只重新解析已有的 golden_frames.txt 并自检
python3 gen_golden.py --check

# 写到指定路径
python3 gen_golden.py --out <输出文件>
```

改了 PROTOCOL.md 里的帧定义之后，要同时改 gen_golden.py，重新生成，再让四方的测试都跑一遍。golden_frames.txt 不要手改。

## 被谁引用

本仓库没有 package.xml，不是 ROS 包。这是刻意的：两个 ROS 仓库会同时把它作为 submodule 带进同一个 colcon 工作区，带 package.xml 就会包名重复。

各仓库都把它放在自己根目录的 protocol/ 下：

- gimbal-edu-tools：robo_bridge_edu 和 virtual_gimbal 的测试读 protocol/golden_frames.txt。
- gimbal-edu-solutions：gimbal_serial 和 class5_tracking 的测试读 protocol/golden_frames.txt。
- gimbal-edu-firmware：host_test 读 protocol/golden_frames.txt。

引用方式是 git submodule：

```bash
# 在引用方仓库的根目录
git submodule add ../gimbal-edu-protocol.git protocol
```

克隆引用方时要带 --recursive，漏了就补一句 git submodule update --init。测试找不到黄金向量时会跳过，跳过不等于通过。

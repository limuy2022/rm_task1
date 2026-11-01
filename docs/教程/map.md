已按新赛程修改程序和指南，**中午可以关闭程序、断电，下午无需回到上午原位。**

上午：

```bash
bash tools/start_mapping.bash
```

结束前另开终端保存：

```bash
bash tools/save_mapping.bash
```

保存成功后备份 `maps/`，再关闭程序。

下午：

```bash
bash tools/start_match.bash
```

程序自动加载上午地图并重定位；匹配不唯一时，用 RViz 指定大致位置和车头方向。裁判给ID后执行：

```bash
bash tools/set_target.bash 0
```

确认重定位、跟踪正常后通知裁判开赛，下午统一提交地图和CSV。

详细步骤：[比赛操作指南](/home/limuy/桌面/code/rm_task1/docs/教程/比赛启动与收尾.md)。

**上NUC前必须重新编译安装。** 657项Python、49项C++、19项离线核验及修正后的跨会话测试通过；完整9000拍长测未通过，15分钟实时性仍需现场确认。

现在的流程是：**上午边建粗图、边录原始数据 → 中午可离线重新加工 → 下午加载选定地图并重定位。**

1. **上午启动建图。**

   底盘先保持静止，在NUC运行：

   ```bash
   cd ~/ws_championship/src/43-MyDuo
   bash tools/start_mapping.bash
   ```

   这个终端保持运行，它会启动雷达、LIO和二维建图，不启动云台运动控制。

2. **另开终端，开始录原始数据。**

   ```bash
   cd ~/ws_championship/src/43-MyDuo
   bash tools/record.bash
   ```

   看到点云、IMU两个话题均已订阅后，**继续静止录5～10秒**，再请现场人员按允许方式移动底盘，覆盖墙角、固定障碍和比赛可能经过的区域。

   此时同时保存两种东西：现场不断生成的粗地图，以及可以反复加工的完整原始bag。

3. **上午结束前，先停止录制，再保存地图。**

   先在录制终端按 **Ctrl+C**，等“原始数据保存并校验完成”。

   然后另开终端执行：

   ```bash
   cd ~/ws_championship/src/43-MyDuo
   bash tools/save_mapping.bash
   ```

   保存成功后，再关闭建图主终端。备份：

   - `records/raw_*`：原始数据和配置，**整个目录一起复制**。
   - `maps/`：现场地图快照，`maps/current`指向本次保存结果。

   **中午可以关闭所有程序、断电，不用保持上午进程运行。**

4. **中午决定是否重新加工。**

   现场地图满意，可以直接使用。

   如果需要调整地面高度、滤波或栅格参数，就用原始bag重新运行LIO和建图，生成一份新地图；不必回场重新扫描。但原来没扫到的地方，离线加工无法补出来。

   三终端的具体回放命令见：[原始数据录制与离线建图](/home/limuy/桌面/code/rm_task1/docs/教程/原始数据录制与离线建图.md)。

5. **下午加载地图，重新定位。**

   使用上午保存的地图：

   ```bash
   bash tools/start_match.bash
   ```

   使用中午重新加工的地图，先把配套PGM和YAML复制到NUC，例如：

   ```bash
   bash tools/start_match.bash \
     map_yaml:="$PWD/maps/rebuilt_01/43_slam_map.yaml"
   ```

   启动前保持底盘静止、相机水平朝车头。程序会在旧图中重新定位，**不要求回到上午原位**。自动匹配不唯一时，用RViz的 **2D Pose Estimate** 指定大致底盘位置和车头方向。

   确认地图与当前点云重合、指定ID跟踪正常后，再通知裁判开赛。正式地图会按下午开赛起点转换后，与CSV一起提交。
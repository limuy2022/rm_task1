明天按这个顺序操作：**更新并编译 → 启动并等对齐 → 设置裁判指定ID → 确认跟踪 → 裁判开赛 → 检查输出 → 收尾。**

1. **提前把最新代码上传到NUC并重新编译。**

   特别要带上新增的 `tools/set_target.py`；这次修改了C++串口桥，必须重新编译。

   ```bash
   cd ~/ws_championship/src/43-MyDuo
   export TRACK_WS="$PWD"
   source /opt/ros/humble/setup.bash
   source "$TRACK_WS/.livox/env.bash"

   colcon --log-base "$TRACK_WS/log" build \
     --base-paths "$TRACK_WS/src" \
     --build-base "$TRACK_WS/build" \
     --install-base "$TRACK_WS/install" \
     --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
   ```

   如果这台NUC还没有本队Livox驱动，先执行 `bash tools/build_livox.bash`，再执行上面的加载和编译。

2. **启动前，底盘保持静止，相机水平朝车头。**

   确认官方相机已启动，没有另一份本队程序占用串口，然后：

   ```bash
   bash tools/start_match.bash
   ```

   **保持这个终端运行，等首次参考对齐完成。** 雷达、定位、建图和跟踪链由入口一起启动；不需要提前加载地图，也不需要手动开启 `motion_enabled` 或 `continuous`。

3. **裁判通知目标ID后，另开终端设置。**

   例如目标为ID 0：

   ```bash
   cd ~/ws_championship/src/43-MyDuo
   bash tools/set_target.bash 0
   ```

   确认两节点读回都是0，并且云台确实跟踪指定码。**确认跟踪正常、定位已持续输出后，再通知裁判启动客户端。**

4. **比赛中保持程序运行，检查CSV在增长。**

   ```bash
   wc -l ~/ws_championship/output/43.csv
   ```

   开赛前没有CSV是正常的；收到第一拍才创建。客户端删除旧CSV已有处理，无需你手动清理文件。

5. **比赛结束，等地图保存完成再退出。**

   确认输出目录有：

   ```text
   43.csv
   43_map.pgm
   43_map.yaml
   ```

   然后在主终端按 **Ctrl+C**，保留输出和本轮 `records/match_*`。下一轮重新启动程序。

现场最重要的是确认**雷达有数据、串口无占用、相机有图、指定码能跟踪、停车有效**。容器测试已通过，但这些仍需真实设备确认。另外，开赛最初1～2拍可能降级输出，存在少量扣分风险。

详细操作见：[比赛启动与收尾](/home/limuy/桌面/code/rm_task1/docs/教程/比赛启动与收尾.md)。
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
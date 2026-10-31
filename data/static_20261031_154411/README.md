# 现场只读采样证据

summary.json给出采样状态与缺样本，未开赛/未启动节点无数据不等于故障，也不等于验收通过。
measurements.jsonl保存IMU完整数值/四元数、位姿、角点/PnP、epoch、状态和接收到的评分Header。
headers.csv记录源时间/接收年龄；cloud_samples.json只保存点云元数据和抽样点，不保存整帧data。
tf_edges.csv保存实际父子边及变换；graph.json/graph_history.jsonl保存发布节点、类型、QoS、GID。
Humble Python不提供逐消息TF来源GID，不能直接把边和发布者逐一配对。
live_parameters.json是本次只读参数服务响应，type=0/value=null表示该参数未声明；并非确认值为0。
source_configs是本队源码文件参考，不能替代live_parameters证明实际生效配置。
camera_info.json/map_metadata.json仅为收到的元数据，没有图像或地图完整payload。
manual_measurements.yaml中的尺寸、外参、零位、限位等必须实测；null不代表0，不能直接加载为ROS参数。
本采样器不发布控制/评分/TF/传感器消息，不打开串口，不读取参数路径指向的外部配置文件。
补填手工测量后请重新压缩目录；本次自动生成的tar.gz不会随目录后续修改更新。

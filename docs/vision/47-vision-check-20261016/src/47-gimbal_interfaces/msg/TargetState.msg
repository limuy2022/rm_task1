std_msgs/Header header          # frame_id = map
float64 cur_x
float64 cur_y
float64 pred_x                  # t_est + 0.1s 的预测位置
float64 pred_y
float64 vel_x
float64 vel_y
float64 acc_x
float64 acc_y
float64[4] cov_xy               # 位置协方差 2x2 行主序
uint8   track_status            # 0=丢失 1=新鲜观测窗口内 2=观测仍未超时的外推
float64 last_obs_age            # 消息header.stamp距有效图像stamp的ROS秒数；未知为-1
bool    valid

std_msgs/Header header
float64 yaw              # 协议域 rad，绕 +z 逆时针为正
float64 pitch            # 协议域 rad，低头为正  (= PITCH_SIGN * atan2(z_up, horiz))
float64 omega_yaw        # 关节角速度（协议域），由 gz/cos(pitch) 反解
float64 omega_pitch      # 关节角速度（协议域），= gyro.y
float64[3] gyro          # gimbal_link 系 rad/s  (x,y,z)
float64[3] accel         # gimbal_link 系 m/s^2（比力，静止水平 ≈ (0,0,9.8)）
float64[4] quat          # w,x,y,z   world -> gimbal_link，已归一化
uint8   link_state       # 0=IDLE 1=ACTIVE 2=FAULT_SILENT 3=SHUTDOWN
bool    enabled
uint32  frames_ok
uint32  frames_bad
uint32  seq_last

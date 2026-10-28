std_msgs/Header header
float64 yaw              # IMU惯性yaw，rad；不是机械关节角，不得用于机械限位
float64 pitch            # IMU惯性pitch，rad，低头为正；不是机械关节角
float64 omega_yaw        # 关节速度未知时NaN；惯性gyro不能直接当关节速度
float64 omega_pitch      # 关节速度未知时NaN
float64[3] gyro          # gimbal_link 系 rad/s  (x,y,z)
float64[3] accel         # gimbal_link 系 m/s^2（比力，静止水平 ≈ (0,0,9.8)）
float64[4] quat          # w,x,y,z，R_world<-gimbal_link，将gimbal向量转至IMU世界
uint8   link_state       # 0=IDLE 1=ACTIVE 2=FAULT_SILENT 3=SHUTDOWN
bool    enabled
uint32  frames_ok
uint32  frames_bad
uint32  seq_last

std_msgs/Header header          # frame_id = camera_optical_frame
uint32  marker_id
float64 x                       # optical 系（x右 y下 z前），z = 光心深度 z_c
float64 y
float64 z
float64[9] cov_optical          # 3x3 行主序，diag(sigma_lat^2, sigma_lat^2, sigma_depth^2)
float64 depth                   # = z，冗余便于调试
float64 reproj_err_px
bool    valid

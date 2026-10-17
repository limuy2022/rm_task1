# One decoded ArUco observation; pixels and ID belong to the same image stamp.
std_msgs/Header header                 # camera_optical_frame, original image stamp
int32 marker_id                        # decoded ID; -1 means target not selected
geometry_msgs/Point32[] corners        # valid=true: exactly four canonical ArUco corners
bool valid                            # false: target lost/disabled, corners must be empty

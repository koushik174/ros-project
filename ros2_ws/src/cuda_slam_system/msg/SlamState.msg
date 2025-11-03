std_msgs/Header header
uint8 STATE_INITIALIZING=0
uint8 STATE_TRACKING=1
uint8 STATE_LOST=2
uint8 STATE_RELOCALIZATION=3

uint8 state
uint32 tracked_points
uint32 total_keyframes
uint32 total_map_points
float32 tracking_quality
geometry_msgs/Pose current_pose

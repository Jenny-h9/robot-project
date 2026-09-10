# NavigationFeedback.msg
int32 state              # 当前导航状态
float32 distance_remaining  # 剩余距离（米）
PoseStamped current_pose # 当前位置
float32 estimated_time_remaining  # 预估剩余时间（秒）
time timestamp           # 时间戳
# AvoidanceCommand.msg
# 避障控制指令消息

# 头部信息
std_msgs/Header header

# 速度指令
geometry_msgs/Twist command

# 避障状态 (参考 AvoidanceState 枚举)
# 0=NORMAL, 1=DETECTING, 2=AVOIDING, 3=RECOVERING, 4=DISABLED
int32 state

# 障碍物信息
bool obstacle_detected
string obstacle_direction    # 'left', 'right', 'front', 'none'
float32 obstacle_distance    # 障碍物距离（米）

# 原因/描述
string reason                # 避障原因描述
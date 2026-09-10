# NavigationStatus.msg
int32 state        # 导航状态 (IDLE=0, NAVIGATING=1, SUCCEEDED=2, FAILED=3, CANCELLED=4, TIMEOUT=5)
string message     # 状态描述信息
time timestamp     # 时间戳
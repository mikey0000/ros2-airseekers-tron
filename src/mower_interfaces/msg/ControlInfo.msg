std_msgs/Header header

# state
uint32 IDLE            = 0   # 空闲
uint32 RUNING          = 1   # 运行
uint32 ARRIVED         = 2   # 抵达
uint32 PAUSE           = 3   # 暂停
uint32 EXCEPTION       = 4   # 异常

# type
uint32 TYPE_FOLLOW_LINE  =  0    # 跟线
uint32 TYPE_OVER_OBJ     =  1    # 越障
uint32 TYPE_BYPASS_OBJ   =  2    # 绕障
uint32 TYPE_FOLLOW_POINT =  3    # 跟点

uint32 state
uint32 type

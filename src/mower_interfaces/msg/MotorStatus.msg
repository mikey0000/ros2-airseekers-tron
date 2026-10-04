#        // 电机状态
#        int8_t status;     // 电机状态: 0:IDLE, 1:RUNNING, 2:LOCKING, -1:ERROR
#              // 电机错误: 0-无错误, 1-过流, 2-过压, 3-欠压, 4-过热, 5-堵转

int8 STATUS_IDLE = 0
int8 STATUS_RUNNING = 1 # 运行
int8 STATUS_LOCKING = 2 # 锁定
int8 STATUS_ERROR = -1
int8 ERROR_OVERCURRENT = -2 # 过流
int8 ERROR_OVERVOLTAGE = -3 # 过压
int8 ERROR_UNDERVOLTAGE = -4 # 欠压
int8 ERROR_OVERHEAT = -5 # 过热
int8 ERROR_STALL = -6 # 堵转
int8 ERROR_OVERLOAD = -7 # 过载

int8 status

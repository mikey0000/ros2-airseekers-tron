std_msgs/Header header

# mower working status
bool is_cutting      # 刀盘是否正在转动
bool is_moving       # 机器是否正在运动
bool is_cmd_moving   # 机器接收到的命令是否是开始运动命令
bool is_docking_done # 机器人是否已经对接完成, 用于判断是否可以充电
bool is_charging     # 机器正在充电

uint8 IDLE = 0
uint8 BACKING_UP = 1
uint8 bumper_routing_status
bool bumper_routing_enabled

bool battery_gate_open # 电池舱门关闭检测
bool press_module # 压草模块在位检测

bool stop_triggered # 急停检测

bool bumper_triggered # bumper 触发检测
bool left_bumper_triggered
bool right_bumper_triggered

bool rain_triggered # 雨滴触发检测
bool lift_triggered # 抬起检测

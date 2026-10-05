# Vendor mower_msgs/LightMode.msg (ROS 1, src/mower_msgs/msg/LightMode.msg) for ROS 2.
# Values are identical to the vendor enum; names are UPPER_SNAKE because ROS 2 rosidl
# rejects CamelCase constant names (vendor name in the trailing comment).


int8 UNKNOWN = -1  # Unknown
int8 MAPPING = 0  # Mapping
int8 IDLE = 1  # Idle
int8 TASK_START = 2  # TaskStart
int8 TASK_FINISHED = 3  # TaskFinished
int8 TASK_PAUSE = 4  # TaskPause
int8 TASK_TERMINATION = 5  # TaskTermination
int8 POWER_ON = 6  # PowerOn
int8 POWER_OFF = 7  # PowerOff
int8 OTA_ING = 8  # OTAIng
int8 OTA_FINISHED = 9  # OTAFinished
int8 PAIR_ING = 10  # PairIng
int8 PAIR_FINISHED = 11  # PairFinished
int8 FINDING = 12  # Finding
int8 DOCKING = 13  # Docking
int8 CHARGING = 14  # Charging
int8 CHARGE_FINISHED = 15  # ChargeFinished
int8 TASK_ING = 16  # TaskIng

int8 LOW_BATTERY = 17  # LowBattery
int8 MANUAL_INTERVENTION = 18  # ManualIntervention
int8 WARN_SENSOR_TRIGGED = 19  # WarnSensorTrigged
int8 ROBOT_UNUSUAL = 20  # RobotUnusual
int8 SIGNAL_ERROR = 21  # SignalError
int8 OPEN_CUTTER = 22  # OpenCutter
int8 CLOSE_CUTTER = 23  # CloseCutter
int8 REMOTE_CONTROL = 24  # RemoteControl # 遥控模式
int8 PAIR_FAILED = 25  # PairFailed
int8 GENERAL_ERROR_RED = 26  # GeneralErrorRed # 通用错误红灯
int8 GENERAL_NORMAL_WHITE = 27  # GeneralNormalWhite # 通用正常白灯

int8 light_mode

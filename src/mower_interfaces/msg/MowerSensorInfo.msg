std_msgs/Header header

# mcu boards firmware version
# RK3588 <-- /dev/serial_mower --> cutter_board <-- uart  --> chassis_board
string cutter_board_version
string chassis_board_version
string rtk_board_version
string mower_package_version

# 3 buttons on top , from RK3588
# bool key_pause_pressed
# bool key_go_docking_pressed
# bool key_power_short_pressed
# bool key_power_long_pressed
# bool kye_dock_and_pause_pressed
uint8 KEY_NONE = 0
uint8 KEY_WORKING_OR_PAUSE = 1
uint8 KEY_GO_DOCKING = 2
uint8 KEY_POWER_SHORT = 4
uint8 KEY_POWER_LONG = 8
uint8 KEY_DOCK_AND_PAUSE = 16

uint8 key_pressed

# sensors from mower MCU
# mower_drivers/mower_base/mower_sdk/protocol/protocol_definitions.h
#
# struct SensorInfo {
#            uint8_t bumper;     // 碰撞传感器: 0-无碰撞, 1-碰撞
#            uint8_t rain;       // 雨滴传感器: 0-无雨滴, 1-有雨滴
#            uint8_t lift;       // 抬起检测: 0-未抬起, 1-已抬起
#            uint8_t stop;       // 停止检测: 0-未停止, 1-已停止
#        };
bool bumper_triggered
bool rain_triggered
uint16 rain_sensor_value # 雨滴传感器值, 最大应该是4095
bool lift_triggered

# BIG STOP button on top
bool stop_triggered

bool is_fill_light_on # 补光灯

# battery
bool battery_gate_open
uint8 battery_temperature
uint64 battery_error

bool press_module

# mower working status
bool is_cutting
bool is_moving
bool is_cmd_moving
bool is_charging
bool is_docking_done # 机器人是否已经对接完成, 用于判断是否可以充电

# bumper routing status from mower_controller::BumperController
#namespace mower_controller
#{
#    class BumperController
#    {
#        public:
#            enum class State
#            {
#                IDLE,
#                BACKING_UP,
#            };
#
bool bumper_routing_enabled # bumper绕障功能是否开启

uint8 IDLE = 0
uint8 BACKING_UP = 1
uint8 bumper_routing_status

# mower motors info
#  // 电机状态
#  struct MotorStatus {
#      uint8_t status;     // 电机状态: 0-正常, 1-过流, 2-过压, 3-欠压, 4-过热, 5-堵转
#      uint8_t error;      // 电机错误: 0-无错误, 1-过流, 2-过压, 3-欠压, 4-过热, 5-堵转
#  };
#
#  struct MotorInfo {
#      uint8_t id;         // 电机ID
#      uint8_t direction;  // 电机方向: 0-正转, 1-反转
#      uint16_t speed;     // 电机速度: 0-1000
#      MotorStatus status; // 电机状态
#  };
#
#  struct MotorsInfo {
#      MotorInfo cutter_motor; // 割草电机
#      MotorInfo left_motor;   // 左电机
#      MotorInfo right_motor;  // 右电机
#      MotorInfo height_motor; // 升降电机
#  };
uint8 CUTTER_SIZE_UNKNOWN = 0
uint8 CUTTER_SIZE_SMALL=1
uint8 CUTTER_SIZE_BIG=2
uint8 cutter_size
MotorInfo cutter_motor
MotorInfo left_motor
MotorInfo right_motor
MotorInfo height_motor

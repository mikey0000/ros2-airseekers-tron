#        struct MotorInfo {
#            int16_t speed;     // 电机速度 单位：rpm
#            int16_t current;   // 电机电流 单位：10mA
#            int16_t voltage;   // 电机电压 单位：10mV
#            int8_t  temperature;   // 电机温度 单位：度
#            MotorStatus status; // 电机状态
#        };

int16 speed_rpm
int16 current
int16 voltage
int8 temperature
MotorStatus status

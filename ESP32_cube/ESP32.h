#define BUZZER      27 // 定义蜂鸣器引脚为27号引脚
#define VBAT        34 // 定义电池电压检测引脚为34号引脚

#define BRAKE       26 // 定义刹车引脚为26号引脚

#define DIR1        4 // 定义电机1的方向控制引脚为4号引脚
#define PWM1        32 // 定义电机1的PWM信号引脚为32号引脚
#define PWM1_CH     1 // 定义电机1的PWM通道为1

#define DIR2        15 // 定义电机2的方向控制引脚为15号引脚
#define PWM2        25 // 定义电机2的PWM信号引脚为25号引脚
#define PWM2_CH     0 // 定义电机2的PWM通道为0

#define DIR3        5 // 定义电机3的方向控制引脚为5号引脚
#define PWM3        18 // 定义电机3的PWM信号引脚为18号引脚
#define PWM3_CH     2 // 定义电机3的PWM通道为2

#define TIMER_BIT  8 // 定义定时器位数为8位
#define BASE_FREQ  20000 // 定义基础频率为20000Hz

#define MPU6050 0x68 // 定义MPU6050设备地址为0x68
#define ACCEL_CONFIG 0x1C // 定义加速度计配置寄存器地址为0x1C
#define GYRO_CONFIG  0x1B // 定义陀螺仪配置寄存器地址为0x1B

#define PWR_MGMT_1 0x6B // 定义电源管理寄存器1地址为0x6B
#define PWR_MGMT_2 0x6C // 定义电源管理寄存器2地址为0x6C

//Sensor output scaling
#define accSens 0 // 定义加速度计输出比例为0，即2g
#define gyroSens 1 // 定义陀螺仪输出比例为1，即500rad/s

#define EEPROM_SIZE 64 // 定义EEPROM存储空间大小为64字节

float Gyro_amount = 0.1;  // 定义陀螺仪采样时间间隔为0.1秒

bool vertical = false; // 定义是否垂直状态标志位，初始值为false
bool calibrating = false; // 定义是否正在校准状态标志位，初始值为false
bool calibrated = false; // 定义是否已校准状态标志位，初始值为false
int balancing_point = 0; // 定义平衡点位置，初始值为0

float K1 = 160; // 定义PID控制器参数K1，初始值为160
float K2 = 10.50; // 定义PID控制器参数K2，初始值为10.50
float K3 = 0.03; // 定义PID控制器参数K3，初始值为0.03
int loop_time = 10; // 定义循环时间间隔，初始值为10ms

struct OffsetsObj { // 定义偏移量结构体
  int ID1;
  float X1;
  float Y1;
  int ID2;
  float X2;
  float Y2;
  int ID3;
  float X3;
  float Y3;
  int ID4;
  float X4;
  float Y4;
};

OffsetsObj offsets; // 定义偏移量对象

float alpha = 0.74; // 定义低通滤波系数alpha，初始值为0.74

int16_t AcX, AcY, AcZ, GyX, GyY, GyZ, gyroX, gyroY, gyroZ, gyroYfilt, gyroZfilt; // 定义加速度计和陀螺仪数据变量

int16_t GyZ_offset = 0; // 定义陀螺仪Z轴偏移量，初始值为0
int16_t GyY_offset = 0; // 定义陀螺仪Y轴偏移量，初始值为0
int16_t GyX_offset = 0; // 定义陀螺仪X轴偏移量，初始值为0
int32_t GyZ_offset_sum = 0; // 定义陀螺仪Z轴偏移量累加值，初始值为0
int32_t GyY_offset_sum = 0; // 定义陀螺仪Y轴偏移量累加值，初始值为0
int32_t GyX_offset_sum = 0; // 定义陀螺仪X轴偏移量累加值，初始值为0

float robot_angleX, robot_angleY, angleX, angleY; // 定义机器人角度变量
float Acc_angleX, Acc_angleY; // 定义加速度计角度变量
int32_t motor_speed_X; // 定义电机X轴速度，初始值为0
int32_t motor_speed_Y; // 定义电机Y轴速度，初始值为0

long currentT, previousT_1, previousT_2 = 0; // 定义时间变量，用于计算循环时间间隔
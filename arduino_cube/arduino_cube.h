#define PWM_3         3 // 定义PWM_3为3
#define DIR_3         4 // 定义DIR_3为4

#define PWM_1         10 // 定义PWM_1为10
#define DIR_1         2 // 定义DIR_1为2

#define PWM_2         9 // 定义PWM_2为9
#define DIR_2         7 // 定义DIR_2为7

#define BRAKE         8 // 定义BRAKE为8
#define BUZZER        12 // 定义BUZZER为12
#define VBAT          A7 // 定义VBAT为A7

#define MPU6050 0x68          // Device address // 定义MPU6050的设备地址为0x68
#define ACCEL_CONFIG 0x1C     // Accelerometer configuration address // 定义加速度计配置地址为0x1C
#define GYRO_CONFIG 0x1B      // Gyro configuration address // 定义陀螺仪配置地址为0x1B
#define PWR_MGMT_1 0x6B
#define PWR_MGMT_2 0x6C

//Sensor output scaling
#define accSens 0             // 0 = 2g, 1 = 4g, 2 = 8g, 3 = 16g // 定义加速度计输出缩放因子，0表示2g，1表示4g，2表示8g，3表示16g
#define gyroSens 1            // 0 = 250rad/s, 1 = 500rad/s, 2 1000rad/s, 3 = 2000rad/s // 定义陀螺仪输出缩放因子，0表示250弧度/秒，1表示500弧度/秒，2表示1000弧度/秒，3表示2000弧度/秒

float Gyro_amount = 0.996;   // 定义陀螺仪比例因子为0.996

bool vertical = false; // 定义垂直标志位为false
bool calibrating = false; // 定义校准标志位为false
bool calibrated = false; // 定义校准完成标志位为false

int balancing_point = 0; // 定义平衡点为0

float pGain = 150; // 定义比例增益为150
float iGain = 14.00; // 定义积分增益为14.00
float sGain = 0.035; // 定义微分增益为0.035
int loop_time = 10; // 定义循环时间为10

struct OffsetsObj { // 定义一个结构体用于存储偏移量
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

OffsetsObj offsets; // 定义一个OffsetsObj类型的变量offsets

float alpha = 0.6; // 定义alpha为0.6

int16_t  AcX, AcY, AcZ, GyY, GyZ, gyroX, gyroY, gyroZ, gyroYfilt, gyroZfilt; // 定义多个整型变量用于存储加速度计和陀螺仪的数据

int16_t  GyZ_offset = 0; // 定义GyZ_offset为0
int16_t  GyY_offset = 0; // 定义GyY_offset为0
int32_t  GyZ_offset_sum = 0; // 定义GyZ_offset_sum为0
int32_t  GyY_offset_sum = 0; // 定义GyY_offset_sum为0

float robot_angleX, robot_angleY, angleX, angleY; // 定义多个浮点型变量用于存储机器人的角度信息
float Acc_angleX, Acc_angleY;      // 定义两个浮点型变量用于存储加速度计的角度信息
int32_t motor_speed_pwmX; // 定义电机速度控制信号motor_speed_pwmX
int32_t motor_speed_pwmY; // 定义电机速度控制信号motor_speed_pwmY

int bat_divider = 57; // 定义电池电压分压电阻值为57

long currentT, previousT_1, previousT_2 = 0; // 定义三个长整型变量用于存储时间信息
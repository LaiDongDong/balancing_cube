#include "ESP32.h" // 引入ESP32库
#include <Wire.h> // 引入I2C通信库
#include <EEPROM.h> // 引入EEPROM存储库
#include "BluetoothSerial.h" // 引入蓝牙串口通信库

BluetoothSerial SerialBT; // 定义蓝牙串口对象

void setup() {
  Serial.begin(115200); // 初始化串口通信，波特率为115200
  SerialBT.begin("ESP32-Cube-blue"); // 初始化蓝牙设备名称为"ESP32-Cube-blue"
  EEPROM.begin(EEPROM_SIZE); // 初始化EEPROM存储，设置存储大小为EEPROM_SIZE
  pinMode(BUZZER, OUTPUT); // 设置蜂鸣器引脚为输出模式
  pinMode(BRAKE, OUTPUT); // 设置刹车引脚为输出模式
  digitalWrite(BRAKE, HIGH); // 设置刹车引脚为高电平
  
  // 以下代码用于设置电机控制引脚、PWM频率和通道等参数
  pinMode(DIR1, OUTPUT);
  ledcSetup(PWM1_CH, BASE_FREQ, TIMER_BIT);
  ledcAttachPin(PWM1, PWM1_CH);
  Motor1_control(0);
  
  pinMode(DIR2, OUTPUT);
  ledcSetup(PWM2_CH, BASE_FREQ, TIMER_BIT);
  ledcAttachPin(PWM2, PWM2_CH);
  Motor2_control(0);
  
  pinMode(DIR3, OUTPUT);
  ledcSetup(PWM3_CH, BASE_FREQ, TIMER_BIT);
  ledcAttachPin(PWM3, PWM3_CH);
  Motor3_control(0);

  // 从EEPROM中读取校准数据
  EEPROM.get(0, offsets);
  if (offsets.ID1 == 99 && offsets.ID2 == 99 && offsets.ID3 == 99 && offsets.ID4 == 99) calibrated = true;
    else calibrated = false;
    
  delay(2000); // 延时2秒
  digitalWrite(BUZZER, HIGH); // 设置蜂鸣器引脚为高电平
  delay(70); // 延时70毫秒
  digitalWrite(BUZZER, LOW); // 设置蜂鸣器引脚为低电平
  angle_setup(); // 调用角度设置函数
}


void loop() {
  // 获取当前时间（毫秒）
  currentT = millis();

  // 如果当前时间与上一次执行的时间差大于等于循环时间，则执行以下操作
  if (currentT - previousT_1 >= loop_time) {
    // 调用 Tuning() 函数进行调谐
    Tuning();  // derinimui
    // 调用 angle_calc() 函数计算角度
    angle_calc();
    // 根据 balancing_point 的值调整角度和判断是否需要垂直平衡
    if (balancing_point == 1) {
      angleX -= offsets.X1;
      angleY -= offsets.Y1;
      if (abs(angleX) > 8 || abs(angleY) > 8) vertical = false;
    } else if (balancing_point == 2) {
      angleX -= offsets.X2;
      angleY -= offsets.Y2;
      if (abs(angleY) > 5) vertical = false;
    } else if (balancing_point == 3) {
      angleX -= offsets.X3;
      angleY -= offsets.Y3;
      if (abs(angleY) > 5) vertical = false;
    } else if (balancing_point == 4) {
      angleX -= offsets.X4;
      angleY -= offsets.Y4;
      if (abs(angleX) > 5) vertical = false;
    }
    
    // 根据角度判断是否需要快速恢复角度
    if (abs(angleX) < 8 || abs(angleY) < 8) {  // fast restore angle
      Gyro_amount = 0.996; 
    } else {
      Gyro_amount = 0.1;
    }

    // 如果需要垂直平衡且已经校准且不在校准过程中，则执行以下操作
    if (vertical && calibrated && !calibrating) {    
      // 设置 BRAKE 引脚为高电平
      digitalWrite(BRAKE, HIGH);
      // 将陀螺仪数据转换为度/秒
      gyroZ = GyZ / 131.0; // Convert to deg/s
      gyroY = GyY / 131.0; // Convert to deg/s
      gyroX = GyX / 131.0; // Convert to deg/s

      // 对陀螺仪数据进行滤波处理
      gyroYfilt = alpha * gyroY + (1 - alpha) * gyroYfilt;
      gyroZfilt = alpha * gyroZ + (1 - alpha) * gyroZfilt;
      
      // 计算 PWM 值
      int pwm_X = constrain(K1 * angleX + K2 * gyroZfilt + K3 * motor_speed_X, -255, 255);
      int pwm_Y = constrain(K1 * angleY + K2 * gyroYfilt + K3 * motor_speed_Y, -255, 255);
      // 更新电机速度
      motor_speed_X += pwm_X; 
      motor_speed_Y += pwm_Y;
      
      // 根据 balancing_point 的值控制电机
      if (balancing_point == 1) {
        XY_to_threeWay(-pwm_X, -pwm_Y);
      } else if (balancing_point == 2) {
        Motor1_control(pwm_Y);
      } else if (balancing_point == 3) {
        Motor2_control(-pwm_Y);
      } else if (balancing_point == 4) {
        Motor3_control(pwm_X);
      }
    } else {
      // 如果不需要垂直平衡或未校准或在校准过程中，则停止电机并设置 BRAKE 引脚为低电平
      XY_to_threeWay(0, 0);
      digitalWrite(BRAKE, LOW);
      motor_speed_X = 0;
      motor_speed_Y = 0;
    }
    // 更新上一次执行的时间
    previousT_1 = currentT;
  }
  
  // 如果当前时间与上一次执行的时间差大于等于 2000 毫秒，则执行以下操作
  if (currentT - previousT_2 >= 2000) {    
    // 调用 battVoltage() 函数获取电池电压并进行显示
    battVoltage((double)analogRead(VBAT) / 207); 
    // 如果未校准且不在校准过程中，则通过串口发送提示信息
    if (!calibrated && !calibrating) {
      SerialBT.println("first you need to calibrate the balancing points...");
    }
    // 更新上一次执行的时间
    previousT_2 = currentT;
  }
}

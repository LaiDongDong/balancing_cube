#include "arduino_cube.h" // 引入Arduino Cube库
#include <Wire.h> // 引入I2C通信库
#include <EEPROM.h> // 引入EEPROM存储库

void setup() {
  Serial.begin(115200); // 初始化串口通信，波特率为115200

  // 设置定时器1、2和3的寄存器值
  TCCR1A = 0b00000001;
  TCCR1B = 0b00001010;
  TCCR2B = 0b00000010;
  TCCR2A = 0b00000011;

  // 设置引脚模式为输出
  pinMode(DIR_1, OUTPUT);
  pinMode(DIR_2, OUTPUT);
  pinMode(DIR_3, OUTPUT);
  pinMode(BRAKE, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  
  // 控制电机速度为0
  Motor1_control(0);
  Motor2_control(0);
  Motor3_control(0);

  // 从EEPROM中读取校准数据
  EEPROM.get(0, offsets);
  if (offsets.ID1 == 99 && offsets.ID2 == 99 && offsets.ID3 == 99 && offsets.ID4 == 99) calibrated = true;
    else calibrated = false;
  delay(3000); // 延时3秒
  beep(); // 蜂鸣器响铃
  angle_setup(); // 角度传感器初始化
}

void loop() {

  currentT = millis(); // 获取当前时间

  // 如果当前时间与上一次循环的时间差大于等于loop_time，则执行以下操作
  if (currentT - previousT_1 >= loop_time) {
    Tuning(); // 调整平衡点
    angle_calc(); // 计算角度
    
    // 根据平衡点的不同，调整角度和判断是否垂直
    if (balancing_point == 1) {
      angleX -= offsets.X1;
      angleY -= offsets.Y1;
      if (abs(angleX) > 8 || abs(angleY) > 8) vertical = false;
    } else if (balancing_point == 2) {
      angleX -= offsets.X2;
      angleY -= offsets.Y2;
      if (abs(angleY) > 6) vertical = false;
    } else if (balancing_point == 3) {
      angleX -= offsets.X3;
      angleY -= offsets.Y3;
      if (abs(angleY) > 6) vertical = false;
    } else if (balancing_point == 4) {
      angleX -= offsets.X4;
      angleY -= offsets.Y4;
      if (abs(angleX) > 6) vertical = false;
    }
 
    // 如果满足条件，则进行电机控制
    if (vertical && calibrated && !calibrating) {    
      digitalWrite(BRAKE, HIGH); // 刹车开启
      gyroZ = GyZ / 131.0; // 将陀螺仪数据转换为度/秒
      gyroY = GyY / 131.0; // 将陀螺仪数据转换为度/秒

      gyroYfilt = alpha * gyroY + (1 - alpha) * gyroYfilt; // 对陀螺仪数据进行滤波处理
      gyroZfilt = alpha * gyroZ + (1 - alpha) * gyroZfilt; // 对陀螺仪数据进行滤波处理
      
      // 根据LQR算法计算PWM值
      int pwm_X = constrain(pGain * angleX + iGain * gyroZfilt + sGain * motor_speed_pwmX, -255, 255);
      int pwm_Y = constrain(pGain * angleY + iGain * gyroYfilt + sGain * motor_speed_pwmY, -255, 255);
      motor_speed_pwmX += pwm_X; 
      motor_speed_pwmY += pwm_Y;
      
      // 根据平衡点的不同，控制电机运动
      if (balancing_point == 1) {
        XY_to_threeWay(-pwm_X, -pwm_Y);
      } else if (balancing_point == 2) {
        Motor1_control(-pwm_Y);
      } else if (balancing_point == 3) {
        Motor2_control(pwm_Y);
      } else if (balancing_point == 4) {
        Motor3_control(-pwm_X);
      }
    } else {
      balancing_point = 0; // 重置平衡点
      XY_to_threeWay(0, 0); // 停止电机运动
      digitalWrite(BRAKE, LOW); // 刹车关闭
      motor_speed_pwmX = 0; // 清零电机速度
      motor_speed_pwmY = 0; // 清零电机速度
    }
    previousT_1 = currentT; // 更新上一次循环的时间
  }
  
  // 如果当前时间与上一次电池电压检测的时间差大于等于2秒，则执行以下操作
  if (currentT - previousT_2 >= 2000) {    
    battVoltage((double)analogRead(VBAT) / bat_divider); // 读取电池电压并显示在串口上
    if (!calibrated && !calibrating) { // 如果未校准且未进行校准，则提示用户先进行校准
      Serial.println("first you need to calibrate the balancing points...");
    }
    previousT_2 = currentT; // 更新上一次电池电压检测的时间
  }
}

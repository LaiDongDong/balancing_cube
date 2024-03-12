// 定义一个函数，用于向指定设备写入数据
void writeTo(byte device, byte address, byte value) {
  // 开始与设备的通信
  Wire.beginTransmission(device);
  // 写入地址和值
  Wire.write(address);
  Wire.write(value);
  // 结束通信并发送数据
  Wire.endTransmission(true);
}

// 定义一个函数，用于控制蜂鸣器发声
void beep() {
    // 将蜂鸣器设置为高电平，使其发声
    digitalWrite(BUZZER, HIGH);
    // 延时70毫秒
    delay(70);
    // 将蜂鸣器设置为低电平，使其停止发声
    digitalWrite(BUZZER, LOW);
    // 延时80毫秒
    delay(80);
}

// 定义一个函数，用于保存校准数据到EEPROM
void save() {
    // 将offsets数组存储到EEPROM的0地址处
    EEPROM.put(0, offsets);
    // 提交更改，确保数据被写入EEPROM
    EEPROM.commit();
    // 从EEPROM的0地址处读取数据到offsets数组中
    EEPROM.get(0, offsets);
    // 判断offsets数组中的ID1、ID2、ID3、ID4是否都等于99，如果是，则设置calibrated为true
    if (offsets.ID1 == 99 && offsets.ID2 == 99 && offsets.ID3 == 99 && offsets.ID4 == 99) calibrated = true;
    // 设置calibrating为false，表示校准已完成
    calibrating = false;
    // 打印"calibrating off"到串口监视器
    Serial.println("calibrating off");
    // 调用beep函数，发出提示音
    beep();
}

// 定义一个函数，用于设置MPU6050的加速度计和陀螺仪输出比例因子
void angle_setup() {
  // 初始化I2C通信
  Wire.begin();
  // 延时100毫秒
  delay (100);
  // 向MPU6050的PWR_MGMT_1寄存器写入0，使能传感器
  writeTo(MPU6050, PWR_MGMT_1, 0);
  // 向MPU6050的ACCEL_CONFIG寄存器写入加速度计输出比例因子
  writeTo(MPU6050, ACCEL_CONFIG, accSens << 3); // Specifying output scaling of accelerometer
  // 向MPU6050的GYRO_CONFIG寄存器写入陀螺仪输出比例因子
  writeTo(MPU6050, GYRO_CONFIG, gyroSens << 3); // Specifying output scaling of gyroscope
  // 延时100毫秒
  delay (100);

  // 循环1024次，计算GyZ轴的偏移量
  for (int i = 0; i < 1024; i++) {
    angle_calc();
    GyZ_offset_sum += GyZ;
    delay(3);
  }
  // 计算GyZ轴的偏移量平均值
  GyZ_offset = GyZ_offset_sum >> 10;
  // 打印GyZ轴的偏移量值到串口监视器
  Serial.print("GyZ offset value = "); Serial.println(GyZ_offset);
  // 调用beep函数，发出提示音
  beep();
  
  // 循环1024次，计算GyY轴的偏移量
  for (int i = 0; i < 1024; i++) {
    angle_calc();
    GyY_offset_sum += GyY;
    delay(3);
  }
  // 计算GyY轴的偏移量平均值
  GyY_offset = GyY_offset_sum >> 10;
  // 打印GyY轴的偏移量值到串口监视器
  Serial.print("GyY offset value = "); Serial.println(GyY_offset);
  // 调用beep函数，发出提示音
  beep();
  
  // 循环1024次，计算GyX轴的偏移量
  for (int i = 0; i < 1024; i++) {
    angle_calc();
    GyX_offset_sum += GyX;
    delay(3);
  }
  // 计算GyX轴的偏移量平均值
  GyX_offset = GyX_offset_sum >> 10;
  // 打印GyX轴的偏移量值到串口监视器
  Serial.print("GyX offset value = "); Serial.println(GyX_offset);
  // 调用beep函数，发出提示音
  beep();
  // 再次调用beep函数，发出提示音
  beep();
}

void angle_calc() {
  // 开始与MPU6050通信，发送0x43命令
  Wire.beginTransmission(MPU6050);
  Wire.write(0x43);
  Wire.endTransmission(false);
  // 请求从MPU6050读取6个字节的数据
  Wire.requestFrom((int) MPU6050, (int) 6, (int) true);
  // 读取陀螺仪X轴数据
  GyX = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  // 读取陀螺仪Y轴数据
  GyY = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  // 读取陀螺仪Z轴数据
  GyZ = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  // 开始与MPU6050通信，发送0x3B命令
  Wire.beginTransmission(MPU6050);
  Wire.write(0x3B);                  
  Wire.endTransmission(false);
  // 请求从MPU6050读取6个字节的数据
  Wire.requestFrom((int)MPU6050, (int)6, (int)true); 
  // 读取加速度计X轴数据
  AcX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  // 读取加速度计Y轴数据
  AcY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  // 读取加速度计Z轴数据
  AcZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)

  // 减去陀螺仪的偏移值
  GyZ -= GyZ_offset;
  GyY -= GyY_offset;
  GyX -= GyX_offset;

  // 计算机器人的X轴角度
  robot_angleX += GyZ * loop_time / 1000 / 65.536;   
  // 根据加速度计的值计算角度（弧度制）
  Acc_angleX = atan2(AcY, -AcX) * 57.2958;               // angle from acc. values * 57.2958 (deg/rad)
  // 结合陀螺仪和加速度计的角度计算最终角度
  robot_angleX = robot_angleX * Gyro_amount + Acc_angleX * (1.0 - Gyro_amount);

  // 计算机器人的Y轴角度
  robot_angleY += GyY * loop_time / 1000 / 65.536; 
  // 根据加速度计的值计算角度（弧度制）
  Acc_angleY = -atan2(AcZ, -AcX) * 57.2958;              // angle from acc. values * 57.2958 (deg/rad)
  // 结合陀螺仪和加速度计的角度计算最终角度
  robot_angleY = robot_angleY * Gyro_amount + Acc_angleY * (1.0 - Gyro_amount);

  // 将计算出的角度赋值给全局变量
  angleX = robot_angleX;
  angleY = robot_angleY;

  // 根据计算出的角度判断机器人的平衡点，并进行相应的操作
  if (abs(angleX - offsets.X2) < 2 && abs(angleY - offsets.Y2) < 0.6) {
    balancing_point = 2;
    if (!vertical) beep();
    vertical = true;
  } else if (abs(angleX - offsets.X3) < 2 && abs(angleY - offsets.Y3) < 0.6) {
    balancing_point = 3;
    if (!vertical) beep();
    vertical = true;
  } else if (abs(angleX - offsets.X4) < 0.6 && abs(angleY - offsets.Y4) < 2) {
    balancing_point = 4;
    if (!vertical) beep();
    vertical = true;
  } else if (abs(angleX - offsets.X1) < 0.4 && abs(angleY - offsets.Y1) < 0.4) {
    balancing_point = 1;
    if (!vertical) beep();
    vertical = true;
  } 
}

// 定义一个函数，将XY坐标转换为三个电机的PWM值
void XY_to_threeWay(float pwm_X, float pwm_Y) {
  // 根据输入的XY坐标计算三个电机的PWM值
  int16_t m1 = round(0.5 * pwm_X - 0.75 * pwm_Y);
  int16_t m2 = round(0.5 * pwm_X + 0.75 * pwm_Y);
  int16_t m3 = -pwm_X;

  // 限制PWM值在-255到255之间
  m1 = constrain(m1, -255, 255);
  m2 = constrain(m2, -255, 255);
  m3 = constrain(m3, -255, 255);

  // 控制三个电机的PWM值
  Motor1_control(m1);
  Motor2_control(m2);
  Motor3_control(m3);
}

// 定义一个函数，根据电压值控制蜂鸣器
void battVoltage(double voltage) {
  // 如果电压在8V到9.5V之间，打开蜂鸣器；否则关闭蜂鸣器
  if (voltage > 8 && voltage <= 9.5) {
    digitalWrite(BUZZER, HIGH);
  } else {
    digitalWrite(BUZZER, LOW);
  }
}

// 定义一个函数，设置PWM通道的值
void pwmSet(uint8_t channel, uint32_t value) {
  ledcWrite(channel, value);
}

// 定义一个函数，控制第一个电机的PWM值
void Motor1_control(int sp) {
  // 如果PWM值为负数，设置电机方向为反向；否则设置为正向
  if (sp < 0) {
    digitalWrite(DIR1, LOW);
    sp = -sp;
  } else {
    digitalWrite(DIR1, HIGH);
  }
  // 设置PWM通道的值，并限制在0到255之间
  pwmSet(PWM1_CH, sp > 255 ? 255 : 255 - sp);
}

// 定义一个函数，控制第二个电机的PWM值
void Motor2_control(int sp) {
  // 如果PWM值为负数，设置电机方向为反向；否则设置为正向
  if (sp < 0) {
    digitalWrite(DIR2, LOW);
    sp = -sp;
  } else {
    digitalWrite(DIR2, HIGH);
  }
  // 设置PWM通道的值，并限制在0到255之间
  pwmSet(PWM2_CH, sp > 255 ? 255 : 255 - sp);
}

// 定义一个函数，控制第三个电机的PWM值
void Motor3_control(int sp) {
  // 如果PWM值为负数，设置电机方向为反向；否则设置为正向
  if (sp < 0) {
    digitalWrite(DIR3, LOW);
    sp = -sp;
  } else {
    digitalWrite(DIR3, HIGH);
  }
  // 设置PWM通道的值，并限制在0到255之间
  pwmSet(PWM3_CH, sp > 255 ? 255 : 255 - sp);
}

int Tuning() {
  if (!SerialBT.available())  return 0; // 如果串口没有可用数据，返回0
  char param = SerialBT.read();               // 读取参数字节
  if (!SerialBT.available()) return 0; // 如果串口没有可用数据，返回0
  char cmd = SerialBT.read();                 // 读取命令字节
  // Serial.print(param); Serial.println(cmd); // 打印参数和命令

  switch (param) {
    case 'p':
      if (cmd == '+')    K1 += 1; // 如果命令是'+'，则增加K1的值
      if (cmd == '-')    K1 -= 1; // 如果命令是'-'，则减少K1的值
      printValues(); // 打印K1、K2、K3的值
      break;
    case 'i':
      if (cmd == '+')    K2 += 0.05; // 如果命令是'+'，则增加K2的值
      if (cmd == '-')    K2 -= 0.05; // 如果命令是'-'，则减少K2的值
      printValues(); // 打印K1、K2、K3的值
      break;
    case 's':
      if (cmd == '+')    K3 += 0.005; // 如果命令是'+'，则增加K3的值
      if (cmd == '-')    K3 -= 0.005; // 如果命令是'-'，则减少K3的值
      printValues(); // 打印K1、K2、K3的值
      break; 
    case 'c':
      if (cmd == '+' && !calibrating) { // 如果命令是'+'且未进行校准
        calibrating = true; // 设置校准标志为true
        SerialBT.println("calibrating on"); // 发送校准开始信息
      }
      if (cmd == '-' && calibrating)  { // 如果命令是'-'且正在进行校准
        SerialBT.print("X: "); SerialBT.print(robot_angleX); SerialBT.print(" Y: "); SerialBT.println(robot_angleY); // 发送机器人角度信息
        if (abs(robot_angleX) < 10 && abs(robot_angleY) < 10) { // 如果机器人角度满足条件
          offsets.ID1 = 99; // 设置偏移量ID为99
          offsets.X1 = robot_angleX; // 设置X轴偏移量
          offsets.Y1 = robot_angleY; // 设置Y轴偏移量
          SerialBT.println("Vertex OK."); // 发送顶点OK信息
          save(); // 保存偏移量
        } else if (robot_angleX > -45 && robot_angleX < -25 && robot_angleY > -30 && robot_angleY < -10) { // 如果机器人角度满足条件
          offsets.ID2 = 99; // 设置偏移量ID为99
          offsets.X2 = robot_angleX; // 设置X轴偏移量
          offsets.Y2 = robot_angleY; // 设置Y轴偏移量
          SerialBT.println("First edge OK."); // 发送第一条边OK信息
          save(); // 保存偏移量
        } else if (robot_angleX > 20 && robot_angleX < 40 && robot_angleY > -30 && robot_angleY < -10) { // 如果机器人角度满足条件
          offsets.ID3 = 99; // 设置偏移量ID为99
          offsets.X3 = robot_angleX; // 设置X轴偏移量
          offsets.Y3 = robot_angleY; // 设置Y轴偏移量
          SerialBT.println("Second edge OK."); // 发送第二条边OK信息
          save(); // 保存偏移量
        } else if (abs(robot_angleX) < 15 && robot_angleY > 30 && robot_angleY < 50) { // 如果机器人角度满足条件
          offsets.ID4 = 99; // 设置偏移量ID为99
          offsets.X4 = robot_angleX; // 设置X轴偏移量
          offsets.Y4 = robot_angleY; // 设置Y轴偏移量
          SerialBT.println("Third edge OK."); // 发送第三条边OK信息
          save(); // 保存偏移量
        } else {
          SerialBT.println("The angles are wrong!!!"); // 发送角度错误信息
          beep(); // 发出提示音
          beep(); // 发出提示音
        }
      }
      break;              
   }
   return 1; // 返回1表示成功执行
}

void printValues() {
  SerialBT.print("K1: "); SerialBT.print(K1); // 发送K1值
  SerialBT.print(" K2: "); SerialBT.print(K2); // 发送K2值
  SerialBT.print(" K3: "); SerialBT.println(K3,4); // 发送K3值，保留4位小数
}

void writeTo(byte device, byte address, byte value) {
  // 向设备写入数据
  Wire.beginTransmission(device); // 开始与设备的通信
  Wire.write(address); // 写入地址
  Wire.write(value); // 写入值
  Wire.endTransmission(true); // 结束通信并发送数据
}

void beep() {
  digitalWrite(BUZZER, HIGH); // 设置蜂鸣器引脚为高电平
  delay(70); // 延时70毫秒
  digitalWrite(BUZZER, LOW); // 设置蜂鸣器引脚为低电平
  delay(80); // 延时80毫秒
}

void save() {
  EEPROM.put(0, offsets); // 将offsets变量的值存储到EEPROM的0地址
  EEPROM.get(0, offsets); // 从EEPROM的0地址读取数据到offsets变量
  if (offsets.ID1 == 99 && offsets.ID2 == 99 && offsets.ID3 == 99 && offsets.ID4 == 99) calibrated = true; // 如果四个ID都为99，则表示已校准
  calibrating = false; // 设置校准状态为false
  Serial.println("calibrating off"); // 输出校准关闭信息
  beep(); // 调用beep函数
}

void angle_setup() {
  Wire.begin(); // 初始化I2C总线
  delay (100); // 延时100毫秒
  writeTo(MPU6050, PWR_MGMT_1, 0); // 向MPU6050写入PWR_MGMT_1寄存器的值
  writeTo(MPU6050, ACCEL_CONFIG, accSens << 3); // 向MPU6050写入ACCEL_CONFIG寄存器的值，指定加速度计的输出比例
  writeTo(MPU6050, GYRO_CONFIG, gyroSens << 3); // 向MPU6050写入GYRO_CONFIG寄存器的值，指定陀螺仪的输出比例
  delay (100); // 延时100毫秒

  for (int i = 0; i < 1024; i++) {
    angle_calc(); // 调用angle_calc函数计算角度
    //Serial.println(GyZ);
    GyZ_offset_sum += GyZ; // 累加GyZ的值
    delay(3); // 延时3毫秒
  }
  GyZ_offset = GyZ_offset_sum >> 10; // 计算GyZ的偏移量
  Serial.print("GyZ offset value = "); Serial.println(GyZ_offset); // 输出GyZ的偏移量
  beep(); // 调用beep函数
  
  for (int i = 0; i < 1024; i++) {
    angle_calc(); // 调用angle_calc函数计算角度
    //Serial.println(GyY);
    GyY_offset_sum += GyY; // 累加GyY的值
    delay(3); // 延时3毫秒
  }
  GyY_offset = GyY_offset_sum >> 10; // 计算GyY的偏移量
  Serial.print("GyY offset value = "); Serial.println(GyY_offset); // 输出GyY的偏移量
  beep(); // 调用beep函数
  beep(); // 调用beep函数
}

void angle_calc() {
  // 从MPU6050读取原始加速度和陀螺仪数据
  Wire.beginTransmission(MPU6050); // 开始与MPU6050通信
  Wire.write(0x45); // 写入0x45（GYRO_YOUT_H）和0x46（GYRO_YOUT_L）寄存器的地址
  Wire.endTransmission(false); // 结束通信
  Wire.requestFrom(MPU6050, 6, true);  // 请求6个字节的数据，从MPU6050的0x45和0x46寄存器开始
  GyY = Wire.read() << 8 | Wire.read(); // 读取GyY的值
  GyZ = Wire.read() << 8 | Wire.read(); // 读取GyZ的值

  Wire.beginTransmission(MPU6050); // 开始与MPU6050通信
  Wire.write(0x3B);                  // 写入0x3B（ACCEL_XOUT_H）和0x3C（ACCEL_XOUT_L）寄存器的地址
  Wire.endTransmission(false); // 结束通信
  Wire.requestFrom(MPU6050, 6, true);  // 请求6个字节的数据，从MPU6050的0x3B和0x3C寄存器开始
  AcX = Wire.read() << 8 | Wire.read(); // 读取AcX的值
  AcY = Wire.read() << 8 | Wire.read(); // 读取AcY的值
  AcZ = Wire.read() << 8 | Wire.read(); // 读取AcZ的值

  GyZ -= GyZ_offset; // 减去GyZ的偏移量
  GyY -= GyY_offset; // 减去GyY的偏移量

  robot_angleX += GyZ * loop_time / 1000 / 65.536; // 计算机器人在X轴上的角度
  Acc_angleX = atan2(AcY, -AcX) * 57.2958; // 计算加速度计在X轴上的角度
  robot_angleX = robot_angleX * Gyro_amount + Acc_angleX * (1.0 - Gyro_amount); // 结合陀螺仪和加速度计的角度计算机器人在X轴上的角度

  robot_angleY += GyY * loop_time / 1000 / 65.536; // 计算机器人在Y轴上的角度
  Acc_angleY = atan2(-AcZ, AcX) * 57.2958; // 计算加速度计在Y轴上的角度
  robot_angleY = robot_angleY * Gyro_amount + Acc_angleY * (1.0 - Gyro_amount); // 结合陀螺仪和加速度计的角度计算机器人在Y轴上的角度

  angleX = robot_angleX; // 更新角度X的值
  angleY = robot_angleY; // 更新角度Y的值

  if (abs(angleX - offsets.X1) < 0.4 && abs(angleY - offsets.Y1) < 0.4) { // 如果角度接近第一个顶点
    balancing_point = 1; // 设置平衡点为1
    if (!vertical) beep(); // 如果机器人没有垂直站立，则调用beep函数
    vertical = true; // 设置机器人垂直站立状态为true
  } else if (abs(angleX - offsets.X2) < 3 && abs(angleY - offsets.Y2) < 0.6) { // 如果角度接近第二个顶点
    balancing_point = 2; // 设置平衡点为2
    if (!vertical) beep(); // 如果机器人没有垂直站立，则调用beep函数
    vertical = true; // 设置机器人垂直站立状态为true
  } else if (abs(angleX - offsets.X3) < 6 && abs(angleY - offsets.Y3) < 0.6) { // 如果角度接近第三个顶点
    balancing_point = 3; // 设置平衡点为3
    if (!vertical) beep(); // 如果机器人没有垂直站立，则调用beep函数
    vertical = true; // 设置机器人垂直站立状态为true
  } else if (abs(angleX - offsets.X4) < 0.6 && abs(angleY - offsets.Y4) < 3) { // 如果角度接近第四个顶点
    balancing_point = 4; // 设置平衡点为4
    if (!vertical) beep(); // 如果机器人没有垂直站立，则调用beep函数
    vertical = true; // 设置机器人垂直站立状态为true
  }
}



void XY_to_threeWay(float pwm_X, float pwm_Y) {
  // 根据输入的pwm_X和pwm_Y计算三个电机的PWM值
  int16_t m1 = round(0.5 * pwm_X - 0.75 * pwm_Y); 
  int16_t m2 = round(0.5 * pwm_X + 0.75 * pwm_Y);
  int16_t m3 = pwm_X;  

  // 将计算出的PWM值限制在-255到255之间
  m1 = constrain(m1, -255, 255);
  m2 = constrain(m2, -255, 255);
  m3 = constrain(m3, -255, 255);
  
  // 控制三个电机
  Motor1_control(-m1);
  Motor2_control(-m2);
  Motor3_control(m3);
}

void battVoltage(double voltage) {
  // 如果电压在8V到9.5V之间，打开蜂鸣器；否则关闭蜂鸣器
  if (voltage > 8 && voltage <= 9.5) {
    digitalWrite(BUZZER, HIGH);
  } else {
    digitalWrite(BUZZER, LOW);
  }
}

void Motor1_control(int sp) {
  // 根据输入的速度值sp设置电机1的方向和PWM值
  if (sp > 0) digitalWrite(DIR_1, LOW);
    else digitalWrite(DIR_1, HIGH);
  analogWrite(PWM_1, 255 - abs(sp));
}

void Motor2_control(int sp) {
  // 根据输入的速度值sp设置电机2的方向和PWM值
  if (sp > 0) digitalWrite(DIR_2, LOW);
    else digitalWrite(DIR_2, HIGH);
  analogWrite(PWM_2, 255 - abs(sp));
}

void Motor3_control(int sp) {
  // 根据输入的速度值sp设置电机3的方向和PWM值
  if (sp > 0) digitalWrite(DIR_3, LOW);
    else digitalWrite(DIR_3, HIGH);
  analogWrite(PWM_3, 255 - abs(sp));
}


int Tuning() {
  // 通过串口接收参数字节和命令字节，根据参数字节执行相应的操作
  if (!Serial.available())  return 0; // 如果串口没有可用数据，返回0
  delay(2); // 延时2毫秒
  char param = Serial.read();               // 获取参数字节
  if (!Serial.available()) return 0; // 如果串口没有可用数据，返回0
  char cmd = Serial.read();                 // 获取命令字节
  Serial.flush(); // 清空串口缓冲区

  switch (param) {
    case 'p':
      if (cmd == '+')    pGain += 1; // 如果命令是'+'，则增加pGain的值
      if (cmd == '-')    pGain -= 1; // 如果命令是'-'，则减少pGain的值
      printValues(); // 打印当前值
      break;
    case 'i':
      if (cmd == '+')    iGain += 0.05; // 如果命令是'+'，则增加iGain的值
      if (cmd == '-')    iGain -= 0.05; // 如果命令是'-'，则减少iGain的值
      printValues(); // 打印当前值
      break;
    case 's':
      if (cmd == '+')    sGain += 0.005; // 如果命令是'+'，则增加sGain的值
      if (cmd == '-')    sGain -= 0.005; // 如果命令是'-'，则减少sGain的值
      printValues(); // 打印当前值
      break;  
    case 'b':
      if (cmd == '+')    bat_divider += 1; // 如果命令是'+'，则增加bat_divider的值
      if (cmd == '-')    bat_divider -= 1; // 如果命令是'-'，则减少bat_divider的值
      printValues(); // 打印当前值
      break;
    case 'c':
      if (cmd == '+' && !calibrating) { // 如果命令是'+'且未进行校准
        calibrating = true; // 设置校准标志为true
        Serial.println("calibrating on"); // 输出校准开始信息
      }
      if (cmd == '-' && calibrating)  { // 如果命令是'-'且正在进行校准
        Serial.print("X: "); Serial.print(robot_angleX); Serial.print(" Y: "); Serial.println(robot_angleY); // 输出机器人的X和Y角度
        if (abs(robot_angleX) < 10 && abs(robot_angleY) < 10) { // 如果X和Y角度都小于10度
          offsets.ID1 = 99; // 设置ID1为99
          offsets.X1 = robot_angleX; // 设置X1为机器人的X角度
          offsets.Y1 = robot_angleY; // 设置Y1为机器人的Y角度
          Serial.println("Vertex OK."); // 输出顶点校准成功信息
          save(); // 保存校准结果
        } else if (robot_angleX > -45 && robot_angleX < -25 && robot_angleY > -30 && robot_angleY < -10) { // 如果X角度在-45到-25之间，Y角度在-30到-10之间
          offsets.ID2 = 99; // 设置ID2为99
          offsets.X2 = robot_angleX; // 设置X2为机器人的X角度
          offsets.Y2 = robot_angleY; // 设置Y2为机器人的Y角度
          Serial.println("First edge OK."); // 输出第一条边校准成功信息
          save(); // 保存校准结果
        } else if (robot_angleX > 20 && robot_angleX < 40 && robot_angleY > -30 && robot_angleY < -10) { // 如果X角度在20到40之间，Y角度在-30到-10之间
          offsets.ID3 = 99; // 设置ID3为99
          offsets.X3 = robot_angleX; // 设置X3为机器人的X角度
          offsets.Y3 = robot_angleY; // 设置Y3为机器人的Y角度
          Serial.println("Second edge OK."); // 输出第二条边校准成功信息
          save(); // 保存校准结果
        } else if (abs(robot_angleX) < 15 && robot_angleY > 30 && robot_angleY < 50) { // 如果X角度小于15度，Y角度在30到50之间
          offsets.ID4 = 99; // 设置ID4为99
          offsets.X4 = robot_angleX; // 设置X4为机器人的X角度
          offsets.Y4 = robot_angleY; // 设置Y4为机器人的Y角度
          Serial.println("Third edge OK."); // 输出第三条边校准成功信息
          save(); // 保存校准结果
        } else {
          Serial.println("The angles are wrong!!!"); // 输出角度错误信息
          beep(); // 发出提示音
          beep(); // 再次发出提示音
        }
      }
      break;                
   }
}

void printValues() {
  // 打印当前的PID参数和电池分压系数
  Serial.print("P: "); Serial.print(pGain);
  Serial.print(" I: "); Serial.print(iGain);
  Serial.print(" S: "); Serial.println(sGain,4);
  Serial.print("Bat_divider: "); Serial.println(bat_divider);
}

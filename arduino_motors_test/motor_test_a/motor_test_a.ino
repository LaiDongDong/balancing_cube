#include <basicMPU6050.h>

#include "motor_test_a.h" // 引入Arduino Cube库
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

  }

void loop() {

  currentT = millis(); // 获取当前时间
  digitalWrite(BRAKE, HIGH); 
  Motor2_control(100);
  delay(2000);
  digitalWrite(BRAKE, LOW);
  delay(1000);
  digitalWrite(BRAKE, HIGH);
  Motor2_control(-100);
  delay(2000);
  Serial.write("OK");

}

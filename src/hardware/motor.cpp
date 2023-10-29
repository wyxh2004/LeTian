#include <motor.h>

float left_motor_angel,left_motor_speed;
float right_motor_angel,right_motor_speed;

void read_left_motor()
{
  Serial1.println("A");
  while(Serial1.available() == 0){}
  left_motor_angel = -Serial1.parseFloat();
  //Serial.print("left");
  //Serial.print(left_motor_angel);
  //Serial.print("    ");

  Serial1.println("S");
  while(Serial1.available() == 0){}
  left_motor_speed = -Serial1.parseFloat();
  //Serial.println(left_motor_speed);
}

void read_right_motor()
{
  Serial2.println("A");
  while(Serial2.available() == 0){}
  right_motor_angel = Serial2.parseFloat();
  //Serial.print("right");
  //Serial.print(right_motor_angel);
  //Serial.print("    ");

  Serial2.println("S");
  while(Serial2.available() == 0){}
  right_motor_speed = Serial2.parseFloat();
  //Serial.println(right_motor_speed);
}

void set_speed(float left_speed,float right_speed)
{
  left_speed = -left_speed;
  Serial1.print("T");
  Serial1.println(-left_speed);

  Serial2.print("T");
  Serial2.println(-right_speed);
}












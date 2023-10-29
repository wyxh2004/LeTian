#include <servo.h>


void ServoAnalogWrite(uint8_t channel, uint32_t value)
 {
  //SERVO  SIGNAL 0 20ms  0.5-2.5ms   0-270度（0-180度）
  //0 4096                2.5% 12.5%   102.4-512   102-514
  // calculate duty, 4095 from 2 ^ 12 - 1
  ledcWrite(channel, value);
}

void servo_init(void)
{
  // Setup timer and attach timer to a servo pin
  ledcSetup(SERVO1_CHANNEL_0, SERVO_BASE_FREQ, SERVO_TIMER_12_BIT);
  ledcAttachPin(SERVO1_PIN, SERVO1_CHANNEL_0);
  ledcSetup(SERVO0_CHANNEL_0, SERVO_BASE_FREQ, SERVO_TIMER_12_BIT);
  ledcAttachPin(SERVO2_PIN, SERVO0_CHANNEL_0);
  // set the brightness on LEDC channel 0
  ServoAnalogWrite(SERVO0_CHANNEL_0, 307+25);//102-514
  ServoAnalogWrite(SERVO1_CHANNEL_0, 307-25);//0 255
}

void servo_set_angel(int servo_left,int servo_right)
{
          //ServoAnalogWrite(SERVO0_CHANNEL_0, 307+servo_vaule);//102-514
          //ServoAnalogWrite(SERVO1_CHANNEL_0, 307-servo_vaule);//0 255
          ServoAnalogWrite(SERVO0_CHANNEL_0, servo_left);//102-514
          ServoAnalogWrite(SERVO1_CHANNEL_0, servo_right);//0 255
}

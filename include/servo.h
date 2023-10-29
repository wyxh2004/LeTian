#ifndef  __SERVO_H
#define __SERVO_H

#include <Arduino.h>

//SERVO init
// use first channel of 16 channels (started from zero)
#define SERVO0_CHANNEL_0     0
#define SERVO1_CHANNEL_0     1
// use 12 bit precission for SERVO timer
#define SERVO_TIMER_12_BIT  12

// use 50Hz as a SERVO base frequency
#define SERVO_BASE_FREQ     50

#define SERVO1_PIN            7
#define SERVO2_PIN            8

void servo_set_angel(int servo_left,int servo_right);
void ServoAnalogWrite(uint8_t channel, uint32_t value);
void servo_init(void);







#endif


#ifndef  __BLINK_BLE_H
#define  __BLINK_BLE_H

#include <Arduino.h>

extern int servo_vaule;
extern float throttle,steering;
extern int servo_left_vaule,servo_right_vaule;

void RAN_CUR_callback(int32_t value_CUR);
void RAN_CUR2_callback(int32_t value_CUR);
void joystick1_callback(uint8_t xAxis, uint8_t yAxis);





#endif


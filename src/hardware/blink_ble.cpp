#include <blink_ble.h>


void RAN_CUR_callback(int32_t value_CUR)
{
    //BLINKER_LOG("get slider value: ", value_CUR);
    servo_vaule = value_CUR;
    Serial.println(servo_vaule);
    servo_left_vaule = 307 + servo_vaule;
    servo_right_vaule = 307 - servo_vaule;
}

void RAN_CUR2_callback(int32_t value_CUR)
{
    //BLINKER_LOG("get slider value: ", value_CUR);
    servo_vaule = value_CUR;
    Serial.println(servo_vaule);
    servo_left_vaule = 307 + servo_vaule;
    servo_right_vaule = 307 + servo_vaule;
}

void joystick1_callback(uint8_t xAxis, uint8_t yAxis)
{
    throttle = (128-yAxis)*0.03;
    steering = (128-xAxis)*0.03;
    Serial.print(throttle);
    Serial.print("         ");
    Serial.println(steering);
}















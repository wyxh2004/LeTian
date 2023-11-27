#include <blink_ble.h>

// RAN_CUR_callback函数用于处理滑块值发生变化的回调函数
void RAN_CUR_callback(int32_t value_CUR)
{
    //BLINKER_LOG("get slider value: ", value_CUR);
    servo_vaule = value_CUR;  // 将滑块值赋给servo_vaule变量
    Serial.println(servo_vaule);  // 在串口监视器中打印servo_vaule的值
    servo_left_vaule = 307 + servo_vaule;  // 根据servo_vaule计算servo_left_vaule的值
    servo_right_vaule = 307 - servo_vaule;  // 根据servo_vaule计算servo_right_vaule的值
}

// RAN_CUR2_callback函数用于处理滑块值发生变化的回调函数
void RAN_CUR2_callback(int32_t value_CUR)
{
    //BLINKER_LOG("get slider value: ", value_CUR);
    servo_vaule = value_CUR;  // 将滑块值赋给servo_vaule变量
    Serial.println(servo_vaule);  // 在串口监视器中打印servo_vaule的值
    servo_left_vaule = 307 + servo_vaule;  // 根据servo_vaule计算servo_left_vaule的值
    servo_right_vaule = 307 + servo_vaule;  // 根据servo_vaule计算servo_right_vaule的值
}

// joystick1_callback函数用于处理遥感器的X轴和Y轴值发生变化的回调函数
void joystick1_callback(uint8_t xAxis, uint8_t yAxis)
{
    throttle = (128-yAxis)*0.03;  // 根据yAxis计算油门值
    steering = (128-xAxis)*0.03;  // 根据xAxis计算方向盘值
    Serial.print(throttle);  // 在串口监视器中打印油门值
    Serial.print("         ");  // 在串口监视器中打印空格
    Serial.println(steering);  // 在串口监视器中打印方向盘值
}















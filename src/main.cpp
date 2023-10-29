#include <Arduino.h>
#include <Wire.h>
#include <servo.h>
#include <motor.h>
#include <main.h>
#include <blink_ble.h>
#include <SimpleFOC.h>
#include <MPU6050_tockn.h>

MPU6050 mpu6050(Wire);              //MPU6050

#define BLINKER_BLE                 //ESP32_BLE
#include <Blinker.h>

BlinkerJoystick JOY1("joy-abc");    //joystick
BlinkerSlider RAN_CUR("ran-abc");   //slider
BlinkerSlider RAN_CUR2("ran-abd");   //slider

// control algorithm parameters
// stabilisation pid
PIDController pid_stb{.P = 0.635, .I = 0.15, .D = 0.005, .ramp = 100000, .limit = 6};
// velocity pid
PIDController pid_vel{.P = 0.6, .I = 0, .D = 0.01, .ramp = 10000, .limit = _PI / 4};
// velocity control filtering
LowPassFilter lpf_pitch_cmd{.Tf = 0.07};
// low pass filters for user commands - throttle and steering
LowPassFilter lpf_throttle{.Tf = 0.5};
LowPassFilter lpf_steering{.Tf = 0.1};

void setup() {

    delay(1000);
    Serial.begin(115200);
    //BLINKER_DEBUG.stream(Serial);         //Blinker_serial_debug

    Blinker.begin();
    JOY1.attach(joystick1_callback);
    RAN_CUR.attach(RAN_CUR_callback);
    RAN_CUR2.attach(RAN_CUR2_callback);

    Serial1.begin(115200,SERIAL_8N1,12,21);  //Serial_motor_left
    Serial2.begin(115200,SERIAL_8N1,13,47);  //Serial_motor_right

    Wire.begin(5,4);                         //mpu6050 IIC
    mpu6050.begin();
   //mpu6050.calcGyroOffsets(true);           //MPU6050校准

    servo_init();                            //servo

    xTaskCreateStaticPinnedToCore(
        task_control, "task_control", 4096, NULL, 2, task_control_stack, &task_control_task_buffer, TASK_RUNNING_CORE_0);

    xTaskCreateStaticPinnedToCore(
        task_motor_move, "task_motor_move", 4096, NULL, 7, task_motor_move_stack, &task_motor_move_task_buffer, TASK_RUNNING_CORE_1);

}

void loop()
{

}

void task_motor_move(void *pvParameters)
{
    while (1)
    {
        set_speed(left_motor,right_motor);                  //set_motor_speed

        Blinker.run();

        servo_set_angel(servo_left_vaule,servo_right_vaule);

        vTaskDelay(2);
    }
}

void task_control(void *pvParameters)
{
    while (1)
    {
        read_left_motor();      //read_left_motor_speed  angel
        read_right_motor();     //read_right_motor_speed  angel
        mpu6050.update();
        // read pitch from the IMU
        float pitch = mpu6050.getAngleX();
        //Serial.println(pitch);
        // calculate the target angle for throttle control
        float target_pitch = lpf_pitch_cmd(pid_vel((left_motor_speed+ right_motor_speed) / 2 ) +lpf_throttle(throttle));
        //更新重心误差
        Offset_parameters = 30 +(servo_vaule - 25)*0.1;
        // calculate the target voltage
        float voltage_control = pid_stb(target_pitch - pitch+Offset_parameters);

        // filter steering
        float steering_adj = lpf_steering(steering);
        // set the tergat voltage value
        left_motor  =  voltage_control-steering_adj;
        right_motor =  voltage_control+steering_adj;

        vTaskDelay(5);
            }

    }




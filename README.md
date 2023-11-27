# 基于esp32的自平衡轮足机器人

## 1. 硬件部分
   本次设计电路部分，全部由立创EDA专业版完成，绘图使用CAD、Solidworks。

（1） **支持芯片**：ESP32-S3-WROOM-1-N8

（2） **引脚引出**：两路舵机接口、两路无刷电机接口、两路IIC接口、四路SPI接口。

（3） **板载MPU6050**：长宽设计为48*48mm 体积小巧

（4） 两路舵机接口，使用XH2.54 3P接口

（5） ESP32集成一键下载电路，使用CH340K串口芯片，使用typec接口与电脑进行通讯和下载程序

###![概念图](https://image.lceda.cn/pullimage/8I11ZmwmBoUxHfqfm31iCeVXagbhuOlryoZlJpcW.jpeg)


##软件部分
####电机驱动采用SimpleFOC算法实现，电机运行在扭矩模式
```
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.torque_controller = TorqueControlType::voltage;
  motor.controller = MotionControlType::torque;
```
####主控与电机驱动使用串口通讯，给定目标值并读取速度和角度值
```
  command.add('T', doTarget, "target voltage");
  command.add('S', sendvel,(char *)"target");     //接收电机的运动指令
  command.add('A', send_angle,(char *)"target");  //接收电机的运动指令
```
####电机第一次初始化motor.initFOC();将函数中参数删除，等待电机上电后串口输出电机转向和偏差角度后填入，下次上电跳过电机自检。
```  
motor.init();
motor.initFOC(5.07,CCW);
```
####主控运行ESP32 RTOS，创建两个任务，串口控制电机和读取电机角度信息
```
xTaskCreateStaticPinnedToCore(task_control,"task_control", 4096, NULL, 2, task_control_stack, &task_control_task_buffer, TASK_RUNNING_CORE_0);

xTaskCreateStaticPinnedToCore(task_motor_move, "task_motor_move", 4096, NULL, 7, task_motor_move_stack, &task_motor_move_task_buffer, TASK_RUNNING_CORE_1);

```

![图片](https://image-pro.lceda.cn/pullimages/4cf5ed3964d1466393e31daf360c837c.webp)



####读取轮速信息，MPU6050信息，进行PID运算
```
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
```
####PID参数调节
```
PIDController pid_stb{.P = 0.635, .I = 0.15, .D = 0.005, .ramp = 100000, .limit = 6};
PIDController pid_vel{.P = 0.6, .I = 0, .D = 0.01, .ramp = 10000, .limit = _PI / 4};

LowPassFilter lpf_pitch_cmd{.Tf = 0.07};
LowPassFilter lpf_throttle{.Tf = 0.5};
LowPassFilter lpf_steering{.Tf = 0.1};
```
#####[项目Github参考链接](https://github.com/WeiYuXingHan/Test-Robot)

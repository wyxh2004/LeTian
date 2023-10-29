#ifndef _MAIN_H
#define _MAIN_H

#define TASK_RUNNING_CORE_0 0
#define TASK_RUNNING_CORE_1 1

int   servo_vaule=-9;
float left_motor,right_motor;
int   servo_left_vaule = 307+servo_vaule,servo_right_vaule = 307-servo_vaule;
float Offset_parameters = 26;               //偏置参数

StaticTask_t task_control_task_buffer;
StackType_t task_control_stack[4096];

StaticTask_t task_motor_move_task_buffer;
StackType_t task_motor_move_stack[4096];


extern float left_motor_angel,left_motor_speed;
extern float right_motor_angel,right_motor_speed;
float  throttle=0,steering=0;

void task_motor_move(void *pvParameters);
void task_control(void *pvParameters);


#endif



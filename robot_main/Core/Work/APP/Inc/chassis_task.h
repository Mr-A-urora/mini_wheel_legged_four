#ifndef __CHASSIS_TASK_H
#define __CHASSIS_TASK_H

#include "main.h"
#include "bsp_motor.h"
#include "pid.h"

#define Joint_Motor_Left_ID 1
#define Joint_Motor_Right_ID 2
#define Wheel_Motor_Left_ID 4
#define Wheel_Motor_Right_ID 3

typedef struct control
{
    float cur;
    float tar;
    float err;
} Control_t;

typedef struct chassis_task
{
    _Motor_Param Jointmotor[2];
    _Motor_Param Wheelmotor[2];
    float Wheel_Motor_K_Matrix[2][4];
    Control_t pitch;
    Control_t d_pitch;
    Control_t x_filter;
    Control_t v_filter;

} _Chassis_Param;

extern _Chassis_Param chassis;
void Chassis_task(void);
#endif

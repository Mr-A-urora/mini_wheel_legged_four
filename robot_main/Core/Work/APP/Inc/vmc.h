#ifndef __VMC_H
#define __VMC_H

#include "main.h"
#include "pid.h"
#define M_PI 3.1415926f

#define Robot_Body_Weight 1.45 / 2 * 9.8
#define Robot_Wheel_Weight 0.3 * 9.8

typedef struct VMC
{
  float theat;

  float Init_Position;
  uint8_t leg_adjust_flag;
  float virtual_leg_length_back;
  float d_virtual_leg_length;
  float virtual_leg_length_set;
  float JointForce_Virtual2Real_K;
  float virtual_support_force_back;
  float virtual_support_force_set;
  uint8_t liftoff_flag;
} VMC_t;

extern VMC_t robot[2];
extern pid_type_def VMC_PID[2];
void vmc_task(void);

float Get_Virtual_Leg_Hight(float Joint_angle);

float Get_JointForce_Virtual2Real_K(float Joint_angle);

#endif

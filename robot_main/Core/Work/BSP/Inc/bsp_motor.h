#ifndef __BSP_MOTOR_H
#define __BSP_MOTOR_H

#include "main.h"

#define JOINT_MOTOR_RX_LEN 8
#define WHEEL_MOTOR_RX_LEN 10
#define constrain(x, max, min) ((x > max) ? (max) : (x < min) ? (min) : (x))

typedef struct
{

    uint8_t id;
    uint8_t mode;
    uint8_t reduction_ratio;
    uint8_t enable_status;

    float iq_buck;
    float iq_set;

    float speed_buck;
    float speed_set;

    float position_buck;
    float position_set;

    float torque_buck;
    float torque_set;
    float torque_K;

    float temperature;
    int error_code;
    uint8_t error_status;
} _Motor_Param;

typedef enum
{
    CONTROL_MODE_CURRENT = 0,
    CONTROL_MODE_CURRENT_RAMP = 1,
    CONTROL_MODE_VELOCITY = 2,
    CONTROL_MODE_VELOCITY_RAMP = 3,
    CONTROL_MODE_POSITION = 4,
    CONTROL_MODE_POSITION_TRAP = 5,
} tControlMode;

void Motorparam_updata(_Motor_Param *Motor, uint8_t *rx_buf, uint8_t rx_buf_len);
void Enable_Motor_Report(uint8_t id);
void Enable_Motor(uint8_t id);
void Disable_Motor(uint8_t id);
void Motor_Setmode(uint8_t id, uint8_t mode);
void jointMotor_SetPosition(uint8_t id, uint8_t traj_vel, uint8_t traj_accel, uint8_t traj_decel, float position);
void Motor_Setiq(uint8_t id, float iq);
float WheelMotor_Torque2Iq(float t);
float JointMotor_Torque2Iq(float t);
float JointMotor_Iq2Torque(float iq);

#endif

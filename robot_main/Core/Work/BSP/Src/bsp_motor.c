#include "bsp_motor.h"
#include "bsp_can.h"
#include "crc8_maxim.h"
#include "usart.h"
#include "chassis_task.h"
#include "arm_math.h"

void Motorparam_updata(_Motor_Param *Motor, uint8_t *rx_buf, uint8_t rx_buf_len)
{
    {
        Motor->position_buck = data_to_float(&rx_buf[0])*2*PI/Motor->reduction_ratio;//rad
        Motor->iq_buck = q510_to_float(&rx_buf[6]);
        Motor->speed_buck = q510_to_float(&rx_buf[4])*2.0f * PI/Motor->reduction_ratio;//rad/s
    }
}

void Enable_Motor_Report(uint8_t id)
{
    uint8_t data[8] = {0};
    FDCAN1_Send_Msg(main_to_M_can | (CAN_CMD_NORMAL_REPORT << 4) | id, data, 1);
}

void Enable_Motor(uint8_t id)
{
    uint8_t data[8] = {0};
    FDCAN1_Send_Msg(main_to_M_can | (CAN_CMD_MOTOR_ENABLE << 4) | id, data, 1);
}

void Disable_Motor(uint8_t id)
{
    uint8_t data[8] = {0};
    FDCAN1_Send_Msg(main_to_M_can | (CAN_CMD_MOTOR_DISABLE << 4) | id, data, 1);
}

void Motor_Setmode(uint8_t id, uint8_t mode)
{
    uint8_t data[8] = {0};
    data[0] = mode;
    FDCAN1_Send_Msg(main_to_M_can | (CAN_CMD_SET_MODE << 4) | id, data, 1);
}

void jointMotor_SetPosition(uint8_t id, uint8_t traj_vel, uint8_t traj_accel, uint8_t traj_decel, float position)
{
    uint8_t data[8];
    data[6] = traj_vel;
    data[5] = traj_accel;
    data[4] = traj_decel;
    float_to_data(position, data);
    FDCAN1_Send_Msg(main_to_M_can | (CAN_CMD_SET_POSITION << 4) | id, data, 8);
}
void Motor_Setiq(uint8_t id, float iq)
{
    uint8_t data[8];
    float_to_data(iq, data);
    FDCAN1_Send_Msg(main_to_M_can | (CAN_CMD_SET_CURRENT << 4) | id, data, 8);
}

/*max 0.4nm k:0.2*/
float WheelMotor_Torque2Iq(float t)
{
    // return t/0.2f;
    return t * 5.0f;
}

float JointMotor_Torque2Iq(float t)
{
    return t  / 0.6f;
}

float JointMotor_Iq2Torque(float iq)
{
    return iq * 0.6f ;
}

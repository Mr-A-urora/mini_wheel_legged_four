#include "chassis_task.h"
#include "cmsis_os.h"
#include "ins_task.h"
#include "rc.h"
#include "vmc.h"
#include "bsp_ext.h"
#include "xbox.h"

uint8_t start_flag = 0;
_Chassis_Param chassis = {
    .Jointmotor[0].id = Joint_Motor_Left_ID,
    .Jointmotor[0].reduction_ratio = 8,

    .Jointmotor[1].id = Joint_Motor_Right_ID,
    .Jointmotor[1].reduction_ratio = 8,

    .Wheelmotor[0].id = Wheel_Motor_Left_ID,
    .Wheelmotor[0].reduction_ratio = 1,

    .Wheelmotor[1].id = Wheel_Motor_Right_ID,
    .Wheelmotor[1].reduction_ratio = 1,

};

float turn_pid_param[3] = {2, 0, -0.2};
pid_type_def TURN_PID;
float yaw_init;
void Chassis_Param_Updata(void);
void Cal_Matrix(float *K_Matrix, float VL_Hight);
uint8_t dx_flag;
// float K_Matrix[4] = {-3.6967, -0.5559, -0.4472, -0.7369};
float K_Matrix[4];
void Chassis_init(void)
{
    /*init motor mode
    enable motor
    init PID
    */
    while (INS.ins_flag == 0)
    { 
        // 等待加速度收敛
        osDelay(1);
    }
    Init_Beep();

    PID_init(&TURN_PID, PID_POSITION, turn_pid_param, 1.5, 0.1);
    Enable_Motor_Report(chassis.Wheelmotor[0].id);
    osDelay(1);
    Enable_Motor_Report(chassis.Wheelmotor[1].id);
    osDelay(1);
    Enable_Motor(chassis.Wheelmotor[0].id);
    osDelay(1);
    Enable_Motor(chassis.Wheelmotor[1].id);
    yaw_init = INS.YawTotalAngle;
}

void Chassis_task(void)
{
    Chassis_init();

    while (1)
    {
        if (xbox.btnStart_falling)
        {
            xbox.btnStart_falling = 0;
            start_flag = !start_flag;
        }
        Chassis_Param_Updata();

        Cal_Matrix(chassis.Wheel_Motor_K_Matrix[0], robot[0].virtual_leg_length_back);
        Cal_Matrix(chassis.Wheel_Motor_K_Matrix[1], robot[1].virtual_leg_length_back);

        chassis.Wheelmotor[0].torque_set = (chassis.Wheel_Motor_K_Matrix[0][0] * chassis.pitch.err + chassis.Wheel_Motor_K_Matrix[0][1] * chassis.d_pitch.err + chassis.Wheel_Motor_K_Matrix[0][2] * chassis.x_filter.err + chassis.Wheel_Motor_K_Matrix[0][3] * chassis.v_filter.err);
        chassis.Wheelmotor[1].torque_set = (chassis.Wheel_Motor_K_Matrix[1][0] * chassis.pitch.err + chassis.Wheel_Motor_K_Matrix[1][1] * chassis.d_pitch.err + chassis.Wheel_Motor_K_Matrix[1][2] * chassis.x_filter.err + chassis.Wheel_Motor_K_Matrix[1][3] * chassis.v_filter.err);
        // chassis.Wheelmotor[0].torque_set = (K_Matrix[0] * chassis.pitch.err + K_Matrix[1] * chassis.d_pitch.err + K_Matrix[2] * chassis.x_filter.err + K_Matrix[3] * chassis.v_filter.err);
        // chassis.Wheelmotor[1].torque_set = (K_Matrix[0] * chassis.pitch.err + K_Matrix[1] * chassis.d_pitch.err + K_Matrix[2] * chassis.x_filter.err + K_Matrix[3] * chassis.v_filter.err);

        Angle_PID_calc(&TURN_PID, INS.YawTotalAngle, yaw_init, INS.Gyro[2]);
        chassis.Wheelmotor[0].iq_set = +(WheelMotor_Torque2Iq(chassis.Wheelmotor[0].torque_set) + TURN_PID.out);
        chassis.Wheelmotor[1].iq_set = -(WheelMotor_Torque2Iq(chassis.Wheelmotor[1].torque_set) - TURN_PID.out);
        if ((robot[0].liftoff_flag & robot[1].liftoff_flag) == 1)
        {
            chassis.Wheelmotor[0].iq_set = 0;
            chassis.Wheelmotor[1].iq_set = 0;
            chassis.x_filter.cur = 0;
        }
        if (start_flag == 1)
        {
            Motor_Setiq(chassis.Wheelmotor[0].id, chassis.Wheelmotor[0].iq_set);
            Motor_Setiq(chassis.Wheelmotor[1].id, chassis.Wheelmotor[1].iq_set);
        }
        else
        {
            yaw_init = INS.YawTotalAngle;
            Motor_Setiq(chassis.Wheelmotor[0].id, 0);
            Motor_Setiq(chassis.Wheelmotor[1].id, 0);
            chassis.x_filter.cur = 0;
        }

        osDelay(2);
    }
}

void Get_Control_Cur_Info(void)
{
    chassis.pitch.cur = -INS.Pitch;
    chassis.d_pitch.cur = -INS.Gyro[0];
    if (dx_flag == 1)
    {
        if (fabsf(chassis.v_filter.cur) < 0.2f)
        {
            dx_flag = 0;
        }
    }
    if (dx_flag)
    {
        chassis.x_filter.cur = 0;
    }
}

void Get_Control_Tar_Info(void)
{
    static uint8_t cnt = 0;

    if (cnt++ > 10)
    {
        cnt = 0;
        yaw_init += -xbox.joyLHori * 0.01 * 0.0004;
    }
    // chassis.pitch.tar = -0.02-0.00016/(robot[0].virtual_leg_length_back+robot[1].virtual_leg_length_back)*0.5;
    chassis.pitch.tar = 0.131;
    chassis.d_pitch.tar = 0;

    chassis.v_filter.tar = xbox.joyLVert * 0.01 * 0.002;
    if (chassis.v_filter.tar != 0)
    {
        dx_flag = 1;
    }
    chassis.v_filter.tar = constrain(chassis.v_filter.tar, 1.5, -1.5);
    chassis.x_filter.tar = 0;
}

void Get_Control_Err_Info(void)
{
    chassis.pitch.err = chassis.pitch.cur - chassis.pitch.tar;
    chassis.d_pitch.err = chassis.d_pitch.cur - chassis.d_pitch.tar;
    chassis.v_filter.err = chassis.v_filter.cur - chassis.v_filter.tar;
    chassis.x_filter.err = chassis.x_filter.cur - chassis.x_filter.tar;
}

void Chassis_Param_Updata(void)
{
    Get_Control_Tar_Info();
    Get_Control_Cur_Info();
    Get_Control_Err_Info();
}

void Cal_Matrix(float *K_Matrix, float VL_Hight)
{
    K_Matrix[0] = 10.4307 * (VL_Hight * VL_Hight) + (-6.1513) * VL_Hight + (-3.7125);
    K_Matrix[1] = (-0.3296) * (VL_Hight * VL_Hight) + (-1.1847) * VL_Hight + (-0.1723);
    K_Matrix[2] = -0.3325;
    K_Matrix[3] = (-11.9945) * (VL_Hight * VL_Hight) + (4.2445) * VL_Hight + (-1.3160);
}

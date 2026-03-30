#include "vmc.h"
#include "cmsis_os.h"
#include "chassis_task.h"
#include "arm_math.h"
#include "math.h"
#include "ins_task.h"
#include "rc.h"
#include "xbox.h"

#define l1 0.12042f
#define l2 0.12168f
#define l3 0.12567f
#define l4 0.05872f
#define l23 0.02958f
extern uint8_t start_flag;
uint8_t jump_flag = 0;
// #define LEG_Position

#define Min_Leg_Hight 0.075f
#define Max_Leg_Hight 0.183f

#define VMC_Locked_IQ 0.0f
#define VMC_Init_Angle 1.256637f /*The Angle between the big_leg and the vertical*/
VMC_t robot[2];
float vmc_pid_param[3] = {500, 0, -5};
pid_type_def VMC_PID[2];
float roll_set = 0;
float roll_pid_param[3] = {-20, 0, 1};
pid_type_def Roll_Contorl_PID;
void VMC_Param_Updata(VMC_t *vmc, _Motor_Param *motor);
void VMC_liftoff_check(void);
uint8_t step = 0;
static uint8_t jump_time = 0;
/*leg mini -0.3662 0.1708 ->1.31772*/
/*leg max -1.4281 1.23156*/
/*leg_hight:0.048~0.217*/
/*Debug leg change rad:0.536923*/
void vmc_task(void)
{
  while (INS.ins_flag == 0)
  { 
    // 等待加速度收敛
    osDelay(1);
  }
  PID_init(&VMC_PID[0], PID_POSITION, vmc_pid_param, 30.0f, 5.0f);
  PID_init(&VMC_PID[1], PID_POSITION, vmc_pid_param, 30.0f, 5.0f);
  PID_init(&Roll_Contorl_PID, PID_POSITION, roll_pid_param, 5.0f, 1.0f);
  for (uint8_t i = 0; i < 5; i++)
  {
    Enable_Motor_Report(chassis.Jointmotor[0].id);
    osDelay(1);
    Enable_Motor_Report(chassis.Jointmotor[1].id);
    osDelay(1);
  }
  robot[0].Init_Position = chassis.Jointmotor[0].position_buck;
  robot[1].Init_Position = chassis.Jointmotor[1].position_buck;
#ifdef LEG_Position
  Motor_Setmode(chassis.Jointmotor[0].id, CONTROL_MODE_POSITION_TRAP);
  osDelay(1);
  Motor_Setmode(chassis.Jointmotor[1].id, CONTROL_MODE_POSITION_TRAP);
  osDelay(1);
  jointMotor_SetPosition(chassis.Jointmotor[0].id, 5, 100, 100, robot[0].Init_Position);
  osDelay(1);
  jointMotor_SetPosition(chassis.Jointmotor[1].id, 5, 100, 100, robot[1].Init_Position);
#endif
  Enable_Motor(chassis.Jointmotor[0].id);
  osDelay(1);
  Enable_Motor(chassis.Jointmotor[1].id);
  osDelay(1);
  while (1)
  {

    if (xbox.btnShare_falling)
    {
      xbox.btnShare_falling = 0;
      jump_flag = 1;
    }
    VMC_PID[0].Kp = VMC_PID[1].Kp = vmc_pid_param[0];
    VMC_PID[0].Ki = VMC_PID[1].Ki = vmc_pid_param[1];
    VMC_PID[0].Kd = VMC_PID[1].Kd = vmc_pid_param[2];

    /*left leg task*/
    VMC_Param_Updata(&robot[0], &chassis.Jointmotor[0]);
    /*right leg task*/
    VMC_Param_Updata(&robot[1], &chassis.Jointmotor[1]);

    VMC_liftoff_check();
    Angle_PID_calc(&Roll_Contorl_PID, INS.Roll, roll_set, INS.Gyro[1]);

#ifdef LEG_Position
    if (rc.sw2 == 3)
    {
      step = 0;
      jointMotor_SetPosition(1, 5, 100, 100, robot[0].Init_Position -= rc.ch2 * 0.0001);

      jointMotor_SetPosition(2, 5, 100, 100, robot[1].Init_Position += rc.ch2 * 0.0001);
    }
    else if (rc.sw2 == 1)
    {
      switch (step)
      {
      case 0:
        jointMotor_SetPosition(1, 50, 100, 100, robot[0].Init_Position - 0.685);

        jointMotor_SetPosition(2, 50, 100, 100, robot[1].Init_Position + 0.685);
        step = 1;
        break;
      case 1:
        if ((robot[0].liftoff_flag & robot[1].liftoff_flag) == 1)
        {
          step = 2;
          jointMotor_SetPosition(1, 50, 100, 100, robot[0].Init_Position);

          jointMotor_SetPosition(2, 50, 100, 100, robot[1].Init_Position);
        }
        break;
      case 2:
        step = 2;
        break;
      default:
        break;
      }
    }
    osDelay(20);
#else
    Angle_PID_calc(&VMC_PID[0], robot[0].virtual_leg_length_back, robot[0].virtual_leg_length_set, robot[0].d_virtual_leg_length);
    Angle_PID_calc(&VMC_PID[1], robot[1].virtual_leg_length_back, robot[1].virtual_leg_length_set, -robot[1].d_virtual_leg_length);

    if (jump_flag == 1)
    {
      chassis.v_filter.err = 0;
      chassis.x_filter.err = 0;
      VMC_PID[0].out = constrain(VMC_PID[0].out, 30.0, -30.0);
      VMC_PID[1].out = constrain(VMC_PID[1].out, 30.0, -30.0);
      chassis.Jointmotor[0].iq_set = JointMotor_Torque2Iq((Robot_Body_Weight + VMC_PID[0].out + Roll_Contorl_PID.out) * robot[0].JointForce_Virtual2Real_K);
      chassis.Jointmotor[1].iq_set = JointMotor_Torque2Iq((Robot_Body_Weight + VMC_PID[1].out - Roll_Contorl_PID.out) * robot[1].JointForce_Virtual2Real_K);
      switch (step)
      {
      case 0:
        robot[0].virtual_leg_length_set = robot[1].virtual_leg_length_set = Min_Leg_Hight;
        Motor_Setiq(chassis.Jointmotor[0].id, -chassis.Jointmotor[0].iq_set);
        Motor_Setiq(chassis.Jointmotor[1].id, chassis.Jointmotor[1].iq_set);
        step = 1;
      case 1:
        if (robot[0].virtual_leg_length_back < Min_Leg_Hight + 0.005 && robot[1].virtual_leg_length_back < Min_Leg_Hight + 0.005)
        {
          jump_time++;
        }
        if (jump_time > 5)
        {
          step = 2;
          jump_time = 0;
        }
        Motor_Setiq(chassis.Jointmotor[0].id, -chassis.Jointmotor[0].iq_set);
        Motor_Setiq(chassis.Jointmotor[1].id, chassis.Jointmotor[1].iq_set);
        break;
      case 2:
        robot[0].virtual_leg_length_set = robot[1].virtual_leg_length_set = Max_Leg_Hight + 0.005;
        Motor_Setiq(chassis.Jointmotor[0].id, -chassis.Jointmotor[0].iq_set);
        Motor_Setiq(chassis.Jointmotor[1].id, chassis.Jointmotor[1].iq_set);
        step = 3;
        break;
      case 3:
        if (robot[0].virtual_leg_length_back > Max_Leg_Hight - 0.005 && robot[1].virtual_leg_length_back > Max_Leg_Hight - 0.005)
        {
          step = 4;
          robot[0].virtual_leg_length_set = robot[1].virtual_leg_length_set = Min_Leg_Hight;
          Motor_Setiq(chassis.Jointmotor[0].id, -chassis.Jointmotor[0].iq_set);
          Motor_Setiq(chassis.Jointmotor[1].id, chassis.Jointmotor[1].iq_set);
        }
        else
        {
          robot[0].virtual_leg_length_set = robot[1].virtual_leg_length_set = Max_Leg_Hight + 0.005;
          Motor_Setiq(chassis.Jointmotor[0].id, -chassis.Jointmotor[0].iq_set);
          Motor_Setiq(chassis.Jointmotor[1].id, chassis.Jointmotor[1].iq_set);
        }
        break;
      case 4:
        Motor_Setiq(chassis.Jointmotor[0].id, -chassis.Jointmotor[0].iq_set);
        Motor_Setiq(chassis.Jointmotor[1].id, chassis.Jointmotor[1].iq_set);
        if (robot[0].virtual_leg_length_back < Min_Leg_Hight + 0.01 && robot[1].virtual_leg_length_back < Min_Leg_Hight + 0.01)
        {
          robot[0].virtual_leg_length_set = robot[1].virtual_leg_length_set = 0.13;
          jump_time++;
        }
        if (jump_time > 5)
        {
          step = 5;
          jump_time = 0;
        }
        break;
      case 5:
        Motor_Setiq(chassis.Jointmotor[0].id, -chassis.Jointmotor[0].iq_set);
        Motor_Setiq(chassis.Jointmotor[1].id, chassis.Jointmotor[1].iq_set);
        jump_flag = 0;
        step = 0;
        break;
      default:
        break;
      }
    }
    else
    {
      if (start_flag == 1)
      {

        VMC_PID[0].out = constrain(VMC_PID[0].out, 20.0, -20.0);
        VMC_PID[1].out = constrain(VMC_PID[1].out, 20.0, -20.0);
        chassis.Jointmotor[0].iq_set = JointMotor_Torque2Iq((Robot_Body_Weight + VMC_PID[0].out + Roll_Contorl_PID.out) * robot[0].JointForce_Virtual2Real_K);
        chassis.Jointmotor[1].iq_set = JointMotor_Torque2Iq((Robot_Body_Weight + VMC_PID[1].out - Roll_Contorl_PID.out) * robot[1].JointForce_Virtual2Real_K);
        robot[0].virtual_leg_length_set += xbox.joyRVert * 0.01 * 0.000001 * 0.5;
        robot[1].virtual_leg_length_set += xbox.joyRVert * 0.01 * 0.000001 * 0.5;
        roll_set = xbox.joyRHori * 0.01 * 0.001;
        roll_set = constrain(roll_set, 0.6, -0.6);
        robot[0].virtual_leg_length_set = constrain(robot[0].virtual_leg_length_set, Max_Leg_Hight, Min_Leg_Hight);
        robot[1].virtual_leg_length_set = constrain(robot[1].virtual_leg_length_set, Max_Leg_Hight, Min_Leg_Hight);

        Motor_Setiq(chassis.Jointmotor[0].id, -chassis.Jointmotor[0].iq_set);
        Motor_Setiq(chassis.Jointmotor[1].id, chassis.Jointmotor[1].iq_set);
      }
      else
      {
        Motor_Setiq(chassis.Jointmotor[0].id, 0);
        Motor_Setiq(chassis.Jointmotor[1].id, 0);
      }
    }
#endif
    /*DEBUG*/
    //    if (rc.sw2 == 3)
    //    {
    //      float position = robot[0].Init_Position - 0.53693 / (2 * PI) * 8;
    //      jointMotor_SetPosition(1, 5, 100, 100, position);
    //      position = robot[1].Init_Position + 0.53693 / (2 * PI) * 8;
    //      jointMotor_SetPosition(2, 5, 100, 100, position);
    //    }
    //    else if (rc.sw2 == 2)
    //    {
    //      jointMotor_SetPosition(1, 50, 100, 100, robot[0].Init_Position);

    //      jointMotor_SetPosition(2, 50, 100, 100, robot[1].Init_Position);
    //    }
    //		osDelay(20);
    /*DEBUG*/

    osDelay(1);
  }
}

float Get_Virtual_Leg_Hight(float Joint_angle)
{
  return 2.0f * (l1 + l2) / 2 * arm_cos_f32(Joint_angle);
  //  double l5 = sqrt(l2 * l2 + l4 * l4 - 2.0f * l2 * l4 * arm_cos_f32(Joint_angle + PI / 4.0f));
  //  double theta2 = asinf(l4 * arm_sin_f32(Joint_angle + PI / 4.0f) / l5);
  //  double theta3 = acosf((l23 * l23 + l5 * l5 - l3 * l3) / (2.0f * l23 * l5));
  //  double theta4 = Joint_angle + theta2 + theta3;
  //  double yc = l2 * arm_sin_f32(Joint_angle) + l1 * arm_sin_f32(theta4);
  //  return yc; // 即L0
}
float Get_Virtual_Leg_d_Hight(float Joint_angle, float d_Joint_angle)
{
  return -2.0f * (l1 + l2) / 2 * arm_sin_f32(Joint_angle) * d_Joint_angle;
}
float Get_JointForce_Virtual2Real_K(float Joint_angle) /*平衡时，支持力等于重力*/
{

  return 2 * (l1 + l2) / 2 * arm_sin_f32(Joint_angle);
  //  double part1 = acosf((l2 * l2 - 2.0f * arm_cos_f32(PI / 4.0f + Joint_angle) * l2 * l4 - l3 * l3 + l4 * l4 + l23 * l23) /
  //                       (2.0f * l23 * pow(l2 * l2 - 2.0f * arm_cos_f32(PI / 4.0f + Joint_angle) * l2 * l4 + l4 * l4, 0.5)));
  //  double part2 = asinf((l4 * arm_sin_f32(Joint_angle + PI / 4.0f)) / pow(l2 * l2 - 2.0f * arm_cos_f32(PI / 4.0f + Joint_angle) * l2 * l4 + l4 * l4, 0.5f));
  //  double part3 = (l4 * arm_cos_f32(Joint_angle + PI / 4.0f)) / pow(l2 * l2 - 2.0f * arm_cos_f32(PI / 4.0f + Joint_angle) * l2 * l4 + l4 * l4, 0.5f);
  //  double part4 = (l2 * l4 * l4 * arm_sin_f32(Joint_angle + PI / 4) * arm_sin_f32(Joint_angle + PI / 4.0f)) / pow(l2 * l2 - 2.0f * arm_cos_f32(PI / 4.0f + Joint_angle) * l2 * l4 + l4 * l4, 1.5f);
  //  double part5 = (l2 * l4 * arm_sin_f32(Joint_angle + PI / 4)) / (l23 * pow(l2 * l2 - 2 * arm_cos_f32(PI / 4.0f + Joint_angle) * l2 * l4 + l4 * l4, 0.5f));
  //  double part6 = (l2 * l4 * arm_sin_f32(Joint_angle + PI / 4) * (l2 * l2 - 2.0f * arm_cos_f32(PI / 4.0f + Joint_angle) * l2 * l4 - l3 * l3 + l4 * l4 + l23 * l23)) /
  //                 (2.0f * l23 * pow(l2 * l2 - 2.0f * arm_cos_f32(PI / 4.0f + Joint_angle) * l2 * l4 + l4 * l4, 1.5f));

  //  double J = l2 * arm_cos_f32(Joint_angle) + l1 * arm_cos_f32(Joint_angle + part1 + part2) *
  //                                                 ((part3 - part4) / pow(1.0f - part4, 0.5f) -
  //                                                  (part5 - part6) / pow(1.0f - pow((l2 * l2 - 2.0f * arm_cos_f32(PI / 4.0f + Joint_angle) * l2 * l4 - l3 * l3 + l4 * l4 + l23 * l23) /
  //                                                                                       (2.0f * l23 * pow(l2 * l2 - 2.0f * arm_cos_f32(PI / 4.0f + Joint_angle) * l2 * l4 + l4 * l4, 0.5f)),
  //                                                                                   2.0f),
  //                                                                        0.5) +
  //                                                  1.0);

  //  return J;
}
void VMC_Param_Updata(VMC_t *vmc, _Motor_Param *motor)
{
  vmc->theat = (VMC_Init_Angle - (fabsf(motor->position_buck - vmc->Init_Position)));
  vmc->virtual_leg_length_back = Get_Virtual_Leg_Hight(vmc->theat);
  vmc->d_virtual_leg_length = Get_Virtual_Leg_d_Hight(vmc->theat, motor->speed_buck);
  vmc->JointForce_Virtual2Real_K = Get_JointForce_Virtual2Real_K(vmc->theat);
  vmc->virtual_support_force_back = JointMotor_Iq2Torque(motor->iq_buck) / vmc->JointForce_Virtual2Real_K;
}

void VMC_liftoff_check(void)
{
  if (robot[0].virtual_support_force_back > 0.5)
  {
    robot[0].liftoff_flag = 1;
  }
  else
  {
    robot[0].liftoff_flag = 0;
  }
  if (robot[1].virtual_support_force_back < -0.5)
  {
    robot[1].liftoff_flag = 1;
  }
  else
  {
    robot[1].liftoff_flag = 0;
  }
  if (step == 2 || step == 3 || step == 4)
  {
    robot[0].liftoff_flag = 1;
    robot[1].liftoff_flag = 1;
  }
}

uint8_t Vmc_Leg_Adjust(void)
{
  // Disable_Jointmotor();
  // osDelay(1);
  // Enable_Jointmotor_Report();
  // osDelay(5);
  // if ((chassis.Jointmotor[0].iq_buck == 0) || ((chassis.Jointmotor[1].iq_buck == 0)))
  // {
  //   Enable_Jointmotor_Report();
  //   osDelay(5);
  //   if ((chassis.Jointmotor[0].iq_buck == 0) || ((chassis.Jointmotor[1].iq_buck == 0)))
  //   {
  //     return 0;
  //   }
  // }
  // JointMotor_Setmode(CONTROL_MODE_POSITION_TRAP);
  // osDelay(2);
  // chassis.Jointmotor[0].position_set = chassis.Jointmotor[0].position_buck;
  // chassis.Jointmotor[1].position_set = chassis.Jointmotor[1].position_buck;
  // jointMotor_SetPosition(1, 5, 100, 100, chassis.Jointmotor[0].position_set);
  // osDelay(1);
  // jointMotor_SetPosition(2, 5, 100, 100, chassis.Jointmotor[1].position_set);
  // Enable_Jointmotor();
  // while ((robot[0].leg_adjust_flag & robot[1].leg_adjust_flag) == 0)
  // {

  //   if ((robot[0].leg_adjust_flag == 0))
  //   {
  //     if (chassis.Jointmotor[0].iq_buck > VMC_Locked_IQ) // locked leg
  //     {
  //       robot[0].Init_Position = chassis.Jointmotor[0].position_buck;
  //       robot[0].leg_adjust_flag = 1;
  //       chassis.Jointmotor[0].position_set = robot[0].Init_Position;
  //       jointMotor_SetPosition(1, 5, 100, 100, chassis.Jointmotor[0].position_set);
  //     }
  //     else
  //     {
  //       jointMotor_SetPosition(1, 5, 100, 100, chassis.Jointmotor[0].position_set);
  //       osDelay(2);
  //       chassis.Jointmotor[0].position_set += 0.01;
  //     }
  //   }

  //   if ((robot[1].leg_adjust_flag == 0))
  //   {
  //     if (chassis.Jointmotor[1].iq_buck > VMC_Locked_IQ) // locked leg
  //     {
  //       robot[1].Init_Position = chassis.Jointmotor[1].position_buck;
  //       robot[1].leg_adjust_flag = 1;
  //       chassis.Jointmotor[1].position_set = robot[1].Init_Position;
  //       jointMotor_SetPosition(2, 5, 100, 100, chassis.Jointmotor[1].position_set);
  //     }
  //     else
  //     {
  //       jointMotor_SetPosition(2, 5, 100, 100, chassis.Jointmotor[1].position_set);
  //       osDelay(2);
  //       chassis.Jointmotor[1].position_set -= 0.01;
  //     }
  //   }
  // }
  return 1;
}

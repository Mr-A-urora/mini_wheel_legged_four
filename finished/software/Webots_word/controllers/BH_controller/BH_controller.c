/*
 * File:          my_controller.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/motor.h>, etc.
 */
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/gyro.h>
#include <webots/inertial_unit.h>
#include <webots/keyboard.h>
#include <webots/camera.h>
#include <webots/position_sensor.h>
#include <stdio.h>
#include <webots/distance_sensor.h>
#include <webots/touch_sensor.h>
#define L 0
#define R 1
#define M_PI 3.14159265358979323846
#define head_weight 1.43 / 2  // 头部重量
#define WHEEL_RADIUS 0.048
#define WHEEL_MOTOR_MAX_TORQUE 1 // 轮子电机最大扭矩
#define WHEEL_Joint_MAX_TORQUE 11  // 电机最大扭矩
#define abs(x) ((x > 0) ? x : (-x))
#define constrain(x, max, min) ((x > max) ? (max) : (x < min) ? (min) \
                                                              : (x))
#define add(x, inc, max) \
  do                     \
  {                      \
    x += inc;            \
    if (x >= max)        \
      x = max;           \
  } while (0) // x变量递增inc这么多,最多达到max值
#define sub(x, dec, min) \
  do                     \
  {                      \
    x -= dec;            \
    if (x <= min)        \
      x = min;           \
  } while (0) // x变量递减dec这么多,最小达到min值
#define l1 0.12042
#define l2 0.12168
#define l3 0.12567
#define l4 0.05872
#define l23 0.02958
/*
 * You may want to add macros here.
 */
#define TIME_STEP 4
typedef enum Devices_Index
{
  Wheel_M_L = 0,
  Wheel_M_R,

  Joint_M_L,
  Joint_M_R,
  Joint_P_L,
  Joint_P_R,
  Gyro,
  Imu,
  wlgyro,
  wrgyro,
  Cam,
  Devices_Num,

} Devices_Index_e;
// 旋转电机
typedef struct Motor
{
  // 转速（单位：弧度/s），不是线速度
  double Velocity;
  // 位置目标值
  double posion;
  // 扭矩目标值
  double Torque;
  // 扭矩反馈
  double Torque_back;

} Motor_t;
typedef struct Pos_Sensor
{
  // 保存偏置的弧度数
  double Bias_Radian;

  // 求走过的弧度数
  double Total_Radian;

  /*用来差分求电机角速度*/
  double This_Radian;
  double Last_Radian;

} Pos_Sensor_t;
// 陀螺仪和角速度计
typedef struct Imu_and_Gyro
{
  double Pitch;
  double Yaw;
  double Roll;

  double Pitch_w;
  double Yaw_w;
  double Roll_W;

  double Pitch_a;
  double Yaw_a;
  double Roll_a;

} Imu_and_Gyro_t;
// 状态变量，变量小写好了方便看是哪个角度
typedef struct Status
{
  double th;
  double dth;
  double x;
  double dx;
  double hight[2];
} Status_t;

// 直腿模型系统输入
typedef struct System_Input
{
  double Wheel_Motor_Torque;

  double Joint_Motor_Torque;

} System_Input_t;

// 底盘的状态变量和系统输入
typedef struct Robot
{

  Status_t cur;
  Status_t Tar;
  Status_t Err;

  System_Input_t System_Input;

} Robot_t;

Motor_t Wheel_Motor[2];
Motor_t Joint_Motor[2];
Pos_Sensor_t Wheel_Pos_Sensor[2];
Pos_Sensor_t Joint_Pos_Sensor[2];
WbDeviceTag Devices[Devices_Num];
Imu_and_Gyro_t Imu_and_Gyro;
Robot_t Robot;

double Joint_kp = -1000, Joint_kd = 10, Joint_t[2];
double Turn_kp=5, Turn_kd=0.5, Turn_torque;
double roll_kp = 300, roll_kd = -80, roll_torque,tar_roll=0;
double yaw_cnt,last_yaw,real_yaw,tar_yaw;
int dx_flag,liftoff_flag;
int jump_flg;
int jump_step = 0;
double Wheel_Motor_K_Matrix[2][4] = {0};
void Get_Device(void)
{
  Devices[Wheel_M_L] = wb_robot_get_device("lwm");
  Devices[Wheel_M_R] = wb_robot_get_device("rwm");

  Devices[Joint_M_L] = wb_robot_get_device("ljm");
  Devices[Joint_M_R] = wb_robot_get_device("rjm");
  Devices[Joint_P_L] = wb_robot_get_device("ljp");
  Devices[Joint_P_R] = wb_robot_get_device("rjp");
  Devices[Gyro] = wb_robot_get_device("gyro");
  Devices[Imu] = wb_robot_get_device("inertial unit");
  Devices[wrgyro] = wb_robot_get_device("rwgyro");
  Devices[wlgyro] = wb_robot_get_device("lwgyro");

  wb_position_sensor_enable(Devices[Joint_P_L], TIME_STEP);
  wb_position_sensor_enable(Devices[Joint_P_R], TIME_STEP);

  wb_gyro_enable(Devices[wrgyro], TIME_STEP);
  wb_gyro_enable(Devices[wlgyro], TIME_STEP);
  wb_gyro_enable(Devices[Gyro], TIME_STEP);

  wb_inertial_unit_enable(Devices[Imu], TIME_STEP);
  wb_keyboard_enable(TIME_STEP);
  wb_motor_enable_torque_feedback(Devices[Joint_M_R], TIME_STEP);
  wb_motor_enable_torque_feedback(Devices[Joint_M_L], TIME_STEP);
  wb_motor_enable_torque_feedback(Devices[Wheel_M_L], TIME_STEP);
  wb_motor_enable_torque_feedback(Devices[Wheel_M_R], TIME_STEP);
}
void Get_Joint_Info(void)
{
  Joint_Pos_Sensor[L].This_Radian = Joint_Pos_Sensor[L].Total_Radian = wb_position_sensor_get_value(Devices[Joint_P_L]);

  Joint_Pos_Sensor[R].This_Radian = Joint_Pos_Sensor[R].Total_Radian = wb_position_sensor_get_value(Devices[Joint_P_R]);

  Joint_Motor[L].Velocity = (Joint_Pos_Sensor[L].This_Radian - Joint_Pos_Sensor[L].Last_Radian) / (TIME_STEP * 0.001);
  Joint_Motor[R].Velocity = (Joint_Pos_Sensor[R].This_Radian - Joint_Pos_Sensor[R].Last_Radian) / (TIME_STEP * 0.001);

  Joint_Pos_Sensor[L].Last_Radian = Joint_Pos_Sensor[L].This_Radian;
  Joint_Pos_Sensor[R].Last_Radian = Joint_Pos_Sensor[R].This_Radian;

  //Joint_Motor[L].Torque_back = wb_motor_get_torque_feedback(Devices[Joint_M_L]);
  //Joint_Motor[R].Torque_back = wb_motor_get_torque_feedback(Devices[Joint_M_R]);
  //printf("Joint_Motor[L].Torque_back %.3f Joint_Motor[R].Torque_back %.3f\n",
  //       Joint_Motor[L].Torque_back, Joint_Motor[R].Torque_back);
  
}
void Get_Imu_and_Gyro_Info(void)
{
  const double *imu_;
  const double *gyro_;
  const double *wlgyro_;
  const double *wrgyro_;
  wlgyro_ = wb_gyro_get_values(Devices[wlgyro]);
  wrgyro_ = wb_gyro_get_values(Devices[wrgyro]);
  imu_ = wb_inertial_unit_get_roll_pitch_yaw(Devices[Imu]); // 返回数组地址，大小是3
  gyro_ = wb_gyro_get_values(Devices[Gyro]);                // 返回数组地址，大小是3
  Imu_and_Gyro.Roll = imu_[0];                              // 横滚
  Imu_and_Gyro.Pitch = imu_[1];                             // 俯仰
  Imu_and_Gyro.Yaw = imu_[2];                               // 偏航

  Imu_and_Gyro.Roll_W = gyro_[0];  // 围绕X方向
  Imu_and_Gyro.Pitch_w = gyro_[1]; // Y方向
  Imu_and_Gyro.Yaw_w = gyro_[2];   // Z方向
  Wheel_Motor[L].Velocity = wlgyro_[2];
  Wheel_Motor[R].Velocity = wrgyro_[2];
}
double Get_Virtual_Leg_Hight(double Joint_angle)
{
  double Virtual_Leg_Hight = 2 * (l1 + l2) / 2 * cos(Joint_angle);
  return Virtual_Leg_Hight;
}
double Get_Virtual_Leg_dHight(double Joint_angle, double dJoint_angle)
{
  double Virtual_Leg_dHight = -2 * (l1 + l2) / 2 * sin(Joint_angle) * dJoint_angle;
  return Virtual_Leg_dHight;
}
void Get_Robot_Cur_Info(void)
{
  Robot.cur.dx = WHEEL_RADIUS * (Wheel_Motor[L].Velocity + Wheel_Motor[R].Velocity) * 0.5;
  Robot.cur.x += Robot.cur.dx * (TIME_STEP * 0.001);

  if (dx_flag == 1)
  {
      if (fabs(Robot.cur.dx) < 0.2)
      {
          dx_flag = 0;
      }
  }
  if (dx_flag)
  {
      Robot.cur.x = 0;
  }
  real_yaw=  Imu_and_Gyro.Yaw;
  if (Imu_and_Gyro.Yaw - last_yaw > M_PI*3/2)
  {
    yaw_cnt--;
  }
  else if (Imu_and_Gyro.Yaw - last_yaw < -M_PI*3/2)
  {
    yaw_cnt++;
  }
  last_yaw = Imu_and_Gyro.Yaw;
  real_yaw+=yaw_cnt * 2 * M_PI;
  Robot.cur.th = Imu_and_Gyro.Pitch;
  Robot.cur.dth = Imu_and_Gyro.Pitch_w;
  Robot.cur.hight[L] = Get_Virtual_Leg_Hight(Joint_Pos_Sensor[L].Total_Radian);
  Robot.cur.hight[R] = Get_Virtual_Leg_Hight(Joint_Pos_Sensor[R].Total_Radian);

}
void Get_Robot_Tar_Info(void)
{
  Robot.Tar.th =0.018;
  Robot.Tar.dth = 0;
}
void Get_Robot_Err_Info(void)
{
  double *cur_tmp, *tar_temp, *err_temp;

  cur_tmp = &Robot.cur.th;
  tar_temp = &Robot.Tar.th;
  err_temp = &Robot.Err.th;

  for (int i = 0; i < 6; i++)
  {
    err_temp[i] = cur_tmp[i] - tar_temp[i];
  }
}
void Cal_Matrix(double *K, double VL_Hight)
{
    K[0] = 10.5193 * (VL_Hight * VL_Hight) + -6.2325 * VL_Hight + -3.7245;
    K[1] = -0.3665 * (VL_Hight * VL_Hight) + -1.1860 * VL_Hight + -0.1730;
    K[2] = -0.6325;
    K[3] = -11.6780 * (VL_Hight * VL_Hight) + 4.1375 * VL_Hight + -1.3043;
}
void WB_Cal_Wheel_Torque(double *k, double *Turn_torque)
{

  Robot.System_Input.Wheel_Motor_Torque = -(k[0] * Robot.Err.th +
                                            k[1] * Robot.Err.dth +
                                            k[2] * Robot.Err.x +
                                            k[3] * Robot.Err.dx);

  Robot.System_Input.Wheel_Motor_Torque = constrain(Robot.System_Input.Wheel_Motor_Torque,
                                                    WHEEL_MOTOR_MAX_TORQUE,
                                                    -WHEEL_MOTOR_MAX_TORQUE);

  *Turn_torque = Robot.System_Input.Wheel_Motor_Torque;
}
void WB_Cal_Joint_Torque(void)
{

  Robot.System_Input.Wheel_Motor_Torque = constrain(Robot.System_Input.Wheel_Motor_Torque,
                                                    WHEEL_Joint_MAX_TORQUE,
                                                    -WHEEL_Joint_MAX_TORQUE);
  Joint_Motor[L].Torque = Robot.System_Input.Wheel_Motor_Torque;
  Joint_Motor[R].Torque = Robot.System_Input.Wheel_Motor_Torque;
}
void WB_Send_Actuator_Commands(void)
{
  wb_motor_set_torque(Devices[Wheel_M_L], Wheel_Motor[L].Torque);
  wb_motor_set_torque(Devices[Wheel_M_R], Wheel_Motor[R].Torque);

  wb_motor_set_torque(Devices[Joint_M_L], Joint_Motor[L].Torque);
  wb_motor_set_torque(Devices[Joint_M_R], Joint_Motor[R].Torque);
}

double Get_Virtual_Joint_k(double Joint_angle)
{
  return -2 * (l1 + l2) * 0.5 * sin(Joint_angle);
}
int press_flag = 0;
void key_updata(void)
{

  int new_key = wb_keyboard_get_key();
  switch (new_key)
  {
  case 'W':
    press_flag = 'W';
    Robot.Tar.dx += 0.02;
    dx_flag = 1;
    break;
  case 'S':
    press_flag = 'S';
    Robot.Tar.dx -= 0.02;
    dx_flag = 1;
    break;
  case 'A':
    press_flag = 'A';
    tar_yaw += 0.01;
    break;
  case 'D':
    press_flag = 'D';
    tar_yaw -= 0.01;
    break;
  case 'J':
    press_flag = 'J';
    break;
  case 314:
    press_flag = 314; // left
    tar_roll = -0.25;
    break;
  case 315:
    press_flag = 315; // up
    Robot.Tar.hight[L] +=0.001;
    Robot.Tar.hight[R] += 0.001;
    break;
  case 316:
    press_flag = 316; // right
    tar_roll = 0.25;
    break;
  case 317:
    press_flag = 317; // down
    Robot.Tar.hight[L] -= 0.001;
    Robot.Tar.hight[R] -= 0.001;
    break;
  default:
    break;
  }
  if (new_key == -1)
  {
    switch (press_flag)
    {
      case 'J':
      jump_flg = 1;
      break;
    }
    press_flag=-1;
    if (Robot.Tar.dx > 0)
    {
      sub(Robot.Tar.dx, 0.02, 0);
    }
    else
    {
      add(Robot.Tar.dx, 0.02, 0);
    }

    tar_roll = 0;
  }
   /*printf*/("new_key %d %d\n",press_flag, jump_flg);
  Robot.Tar.dx = constrain(Robot.Tar.dx, 1.6, -1.6);

}
void jump_task(void)
{

  if (jump_flg==1)
  {
      Robot.Err.dx = 0;
      Robot.Err.x = 0;
  switch (jump_step)
  {
  case 0:

    Robot.Tar.hight[L]=0.06;
    Robot.Tar.hight[R]=0.06;
    jump_step++;
    break;
  case 1:
    if(Robot.cur.hight[L]<0.075 && Robot.cur.hight[R]<0.075)
    {
      Robot.Tar.hight[L]=0.21;
      Robot.Tar.hight[R]=0.21;
      jump_step++;
    }
    break;
  case 2:
    if(Robot.cur.hight[L]>0.185 && Robot.cur.hight[R]>0.185)
    {
      Robot.Tar.hight[L]=0.06;
      Robot.Tar.hight[R]=0.06;
      jump_step++;
    }
    break;
  case 3:
    if(Robot.cur.hight[L]<0.085 && Robot.cur.hight[R]<0.085)
    {
      Robot.Tar.hight[L]=0.14;
      Robot.Tar.hight[R]=0.14;
      jump_step++;
    }
    break;
  case 4:
  jump_flg = 0;
  jump_step=0;
  break;

  default:
    break;
  }
  }
  

}
/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char **argv)
{
  /* necessary to initialize webots stuff */
  wb_robot_init();

  /*
   * You should declare here WbDeviceTag variables for storing
   * robot devices like this:
   *  WbDeviceTag my_sensor = wb_robot_get_device("my_sensor");
   *  WbDeviceTag my_actuator = wb_robot_get_device("my_actuator");
   */
  Get_Device();
  WbDeviceTag touch[2];
  touch[0] = wb_robot_get_device("ltouch");
  touch[1] = wb_robot_get_device("rtouch");
  wb_touch_sensor_enable(touch[0], TIME_STEP);
  wb_touch_sensor_enable(touch[1], TIME_STEP);
  wb_robot_step(TIME_STEP);
  Get_Joint_Info();
  Robot.Tar.hight[L] = Get_Virtual_Leg_Hight(Joint_Pos_Sensor[L].Total_Radian);
  Robot.Tar.hight[R] = Get_Virtual_Leg_Hight(Joint_Pos_Sensor[R].Total_Radian);
  /* main loop
   * Perform simulation steps of TIME_STEP milliseconds
   * and leave the loop when the simulation is over
   */
  while (wb_robot_step(TIME_STEP) != -1)
  {
    /*
     * Read the sensors :
     * Enter here functions to read sensor data, like:
     *  double val = wb_distance_sensor_get_value(my_sensor);
     */
    key_updata();
    Get_Imu_and_Gyro_Info();
    Get_Joint_Info();
    Get_Robot_Cur_Info();
    Get_Robot_Tar_Info();
    Get_Robot_Err_Info();
 
    Cal_Matrix(&Wheel_Motor_K_Matrix[L], Robot.cur.hight[L]);
    Cal_Matrix(&Wheel_Motor_K_Matrix[R], Robot.cur.hight[R]);
    //printf("Lhight %.3f, Rhight %.3f\n",
    //       Robot.cur.hight[L], Robot.cur.hight[R]);
    double touch1 = wb_touch_sensor_get_value(touch[0]);
    double touch2 = wb_touch_sensor_get_value(touch[1]);
    //printf("%.2f,%.2f\n", touch1, touch2);
    jump_task();
    WB_Cal_Wheel_Torque(&Wheel_Motor_K_Matrix[L], &Wheel_Motor[L].Torque);
    WB_Cal_Wheel_Torque(&Wheel_Motor_K_Matrix[R], &Wheel_Motor[R].Torque);
    
    Turn_torque=Turn_kp*(real_yaw-tar_yaw)+Turn_kd*(Imu_and_Gyro.Yaw_w);
    roll_torque= roll_kp * (tar_roll-Imu_and_Gyro.Roll) + roll_kd * Imu_and_Gyro.Roll_W;
    Robot.Tar.hight[L] = constrain(Robot.Tar.hight[L], 0.2, 0.07);
    Robot.Tar.hight[R] = constrain(Robot.Tar.hight[R], 0.2, 0.07);
    Joint_t[L] = Joint_kp * Robot.Err.hight[L] + Joint_kd * Joint_Motor[L].Velocity;
    Joint_t[R] = Joint_kp * Robot.Err.hight[R] + Joint_kd * Joint_Motor[R].Velocity;

    Joint_Motor[L].Torque = Get_Virtual_Joint_k(Joint_Pos_Sensor[L].Total_Radian) * (head_weight * 9.8+ Joint_t[L]+roll_torque);
    Joint_Motor[R].Torque = Get_Virtual_Joint_k(Joint_Pos_Sensor[R].Total_Radian) * (head_weight * 9.8+ Joint_t[R]-roll_torque);

    Wheel_Motor[L].Torque+=Turn_torque;
    Wheel_Motor[R].Torque-=Turn_torque;
    if(touch1==0 && touch2==0)
    {
      //liftoff
      liftoff_flag = 1;
      Robot.cur.dx=0;
      Robot.cur.x=0;
      Robot.Err.dx = 0;
      Robot.Err.x = 0;
      Wheel_Motor[L].Torque = 0;
      Wheel_Motor[R].Torque= 0;
    }
    else
    {
        liftoff_flag = 0;
    }
    // printf("Joint_t[L] %.3f Joint_t[R] %.3f %d %.3f,%.3f\n",
    //     Joint_Motor[L].Torque, Joint_Motor[R].Torque, liftoff_flag, Wheel_Motor[L].Torque, Wheel_Motor[R].Torque);
    WB_Send_Actuator_Commands();

    // printf("Lhight %.3f, Rhight %.3f  %.3f\n",
    //        Robot.cur.hight[L], Robot.cur.hight[R], Joint_t[L]);

     printf("dx %.2f x %.2f th %.2f dth %.2f %.2f\n",
            Robot.Err.dx, Robot.Err.x, Robot.Err.th, Robot.Err.dth, Wheel_Motor[L].Torque);

      // printf("yaw %.2f %.2f  pitch %.2f %.2f roll %.2f %.2f\n", Imu_and_Gyro.Yaw,Imu_and_Gyro.Yaw_w,
      //        Imu_and_Gyro.Pitch,Imu_and_Gyro.Pitch_w,
      //        Imu_and_Gyro.Roll,Imu_and_Gyro.Roll_W);
    // printf("%.2f,%.2f\n",Imu_and_Gyro.Yaw,real_yaw);
    // printf("lwgyro %.2f  rwgyro %.2f \n",Wheel_Motor[L].Velocity, Wheel_Motor[R].Velocity);

    /* Process sensor data here */

    /*
     * Enter here functions to send actuator commands, like:
     * wb_motor_set_position(my_actuator, 10.0);
     */
  };

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}

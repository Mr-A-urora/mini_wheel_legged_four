#include "observe_task.h"

#include "cmsis_os.h"

#include "ins_task.h"
#include "chassis_task.h"

#define Wheel_Radius 0.049f

KalmanFilter_t vaEstimateKF; // 卡尔曼滤波器结构体

float vaEstimateKF_F[4] = {1.0f, 0.003f,
                           0.0f, 1.0f}; // 状态转移矩阵，控制周期为0.001s

float vaEstimateKF_P[4] = {1.0f, 0.0f,
                           0.0f, 1.0f}; // 后验估计协方差初始值

float vaEstimateKF_Q[4] = {0.5f, 0.0f,
                           0.0f, 0.5f}; // Q矩阵初始值

float vaEstimateKF_R[4] = {100.0f, 0.0f,
                           0.0f, 100.0f};

float vaEstimateKF_K[4];

const float vaEstimateKF_H[4] = {1.0f, 0.0f,
                                 0.0f, 1.0f}; // 设置矩阵H为常量

// extern vmc_leg_t right;
// extern vmc_leg_t left;

float vel_acc[2];
uint32_t OBSERVE_TIME = 3; // 任务周期
void Observe_task(void)
{
  while (INS.ins_flag == 0)
  { // 等待加速度收敛
    osDelay(1);
  }
  static float wr, wl = 0.0f;
  static float vrb, vlb = 0.0f;
  static float aver_v = 0.0f;

  xvEstimateKF_Init(&vaEstimateKF);

  while (1)
  {
    //顺时针为正
		 wr = -chassis.Wheelmotor[1].speed_buck - INS.Gyro[0];
		 vrb = wr * Wheel_Radius;

		 wl = -chassis.Wheelmotor[0].speed_buck + INS.Gyro[0];
		 vlb = wl * Wheel_Radius;


		 aver_v = (vrb - vlb) / 2.0f; // 取平均
		 xvEstimateKF_Update(&vaEstimateKF, INS.MotionAccel_n[1], aver_v);

		 //原地自转的过程中v_filter和x_filter应该都是为0
		 chassis.v_filter.cur=vel_acc[0];//得到卡尔曼滤波后的速度
		 chassis.x_filter.cur=chassis.x_filter.cur+chassis.v_filter.cur*((float)OBSERVE_TIME/1000.0f);

    // 如果想直接用轮子速度，不做融合的话可以这样
//    chassis.v_filter.cur = (-chassis.Wheelmotor[1].speed_buck + chassis.Wheelmotor[0].speed_buck) * Wheel_Radius * 0.5f; // 0.0603是轮子半径，电机反馈的是角速度，乘半径后得到线速度，数学模型中定义的是轮子顺时针为正，所以要乘个负号
//    chassis.x_filter.cur = chassis.x_filter.cur + chassis.v_filter.cur * ((float)OBSERVE_TIME / 1000.0f);

    osDelay(OBSERVE_TIME);
  }
}

void xvEstimateKF_Init(KalmanFilter_t *EstimateKF)
{
  Kalman_Filter_Init(EstimateKF, 2, 0, 2); // 状态向量2维 没有控制量 测量向量2维

  memcpy(EstimateKF->F_data, vaEstimateKF_F, sizeof(vaEstimateKF_F));
  memcpy(EstimateKF->P_data, vaEstimateKF_P, sizeof(vaEstimateKF_P));
  memcpy(EstimateKF->Q_data, vaEstimateKF_Q, sizeof(vaEstimateKF_Q));
  memcpy(EstimateKF->R_data, vaEstimateKF_R, sizeof(vaEstimateKF_R));
  memcpy(EstimateKF->H_data, vaEstimateKF_H, sizeof(vaEstimateKF_H));
}

void xvEstimateKF_Update(KalmanFilter_t *EstimateKF, float acc, float vel)
{
  // 卡尔曼滤波器测量值更新
  EstimateKF->MeasuredVector[0] = vel; // 测量速度
  EstimateKF->MeasuredVector[1] = acc; // 测量加速度

  // 卡尔曼滤波器更新函数
  Kalman_Filter_Update(EstimateKF);

  // 提取估计值
  for (uint8_t i = 0; i < 2; i++)
  {
    vel_acc[i] = EstimateKF->FilteredValue[i];
  }
}

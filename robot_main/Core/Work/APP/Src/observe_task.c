#include "observe_task.h"
#include "cmsis_os.h"
#include "ins_task.h"
#include "chassis_task.h"

#define Wheel_Radius 0.049f

KalmanFilter_t vaEstimateKF; // 速度加速度卡尔曼滤波器结构体

float vaEstimateKF_F[4] = {1.0f, 0.003f,
                           0.0f, 1.0f}; // 状态转移矩阵，控制周期为0.001s

float vaEstimateKF_P[4] = {1.0f, 0.0f,
                           0.0f, 1.0f}; // 误差协方差矩阵初始值

float vaEstimateKF_Q[4] = {0.5f, 0.0f,
                           0.0f, 0.5f}; // Q矩阵初始值（过程噪声协方差）

float vaEstimateKF_R[4] = {100.0f, 0.0f,
                           0.0f, 100.0f}; // R矩阵初始值（测量噪声协方差）

float vaEstimateKF_K[4]; // 卡尔曼增益矩阵

const float vaEstimateKF_H[4] = {1.0f, 0.0f,
                                 0.0f, 1.0f}; // 观测矩阵H为单位阵

// extern vmc_leg_t right;
// extern vmc_leg_t left;

float vel_acc[2]; // 滤波后的速度加速度值
uint32_t OBSERVE_TIME = 3; // 观测周期（ms）
void Observe_task(void)
{
  while (INS.ins_flag == 0)
  { 
    // 等待IMU初始化完成
    osDelay(1);
  }
  static float wr, wl = 0.0f;
  static float vrb, vlb = 0.0f;
  static float aver_v = 0.0f;

  xvEstimateKF_Init(&vaEstimateKF);

  while (1)
  {
    // 顺时针为正方向
    wr = -chassis.Wheelmotor[1].speed_buck - INS.Gyro[0];
    vrb = wr * Wheel_Radius;

    wl = -chassis.Wheelmotor[0].speed_buck + INS.Gyro[0];
    vlb = wl * Wheel_Radius;

    aver_v = (vrb - vlb) / 2.0f; // ȡƽ��
    xvEstimateKF_Update(&vaEstimateKF, INS.MotionAccel_n[1], aver_v);

    // 原本旋转的过程中v_filter和x_filter应该都为0
    chassis.v_filter.cur = vel_acc[0]; // 得到经过卡尔曼滤波后的速度
    chassis.x_filter.cur = chassis.x_filter.cur + chassis.v_filter.cur * ((float)OBSERVE_TIME / 1000.0f);

    // 备用方案：直接使用轮速计算速度，作为融合的基础参考
    //    chassis.v_filter.cur = (-chassis.Wheelmotor[1].speed_buck + chassis.Wheelmotor[0].speed_buck) * Wheel_Radius * 0.5f; // 0.0603为轮胎半径，陀螺仪输出的是角速度，乘半径得到线速度，数学模型中的方向规定顺时针为正，此处需要取反
    //    chassis.x_filter.cur = chassis.x_filter.cur + chassis.v_filter.cur * ((float)OBSERVE_TIME / 1000.0f);

    osDelay(OBSERVE_TIME);
  }
}

void xvEstimateKF_Init(KalmanFilter_t *EstimateKF)
{
  Kalman_Filter_Init(EstimateKF, 2, 0, 2); // 状态量2维，无控制量，观测量2维

  memcpy(EstimateKF->F_data, vaEstimateKF_F, sizeof(vaEstimateKF_F));
  memcpy(EstimateKF->P_data, vaEstimateKF_P, sizeof(vaEstimateKF_P));
  memcpy(EstimateKF->Q_data, vaEstimateKF_Q, sizeof(vaEstimateKF_Q));
  memcpy(EstimateKF->R_data, vaEstimateKF_R, sizeof(vaEstimateKF_R));
  memcpy(EstimateKF->H_data, vaEstimateKF_H, sizeof(vaEstimateKF_H));
}

void xvEstimateKF_Update(KalmanFilter_t *EstimateKF, float acc, float vel)
{
  // 设置卡尔曼滤波器测量值输入
  EstimateKF->MeasuredVector[0] = vel; // �����ٶ�
  EstimateKF->MeasuredVector[1] = acc; // �������ٶ�

  // 运行卡尔曼滤波器更新函数
  Kalman_Filter_Update(EstimateKF);

  // 获取滤波值
  for (uint8_t i = 0; i < 2; i++)
  {
    vel_acc[i] = EstimateKF->FilteredValue[i];
  }
}

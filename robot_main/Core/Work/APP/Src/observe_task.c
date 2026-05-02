#include "observe_task.h"

#include "cmsis_os.h"

#include "ins_task.h"
#include "chassis_task.h"

#define Wheel_Radius 0.049f

KalmanFilter_t vaEstimateKF; 

float vaEstimateKF_F[4] = {1.0f, 0.003f,
                           0.0f, 1.0f};

float vaEstimateKF_P[4] = {1.0f, 0.0f,
                           0.0f, 1.0f};

float vaEstimateKF_Q[4] = {0.5f, 0.0f,
                           0.0f, 0.5f};

float vaEstimateKF_R[4] = {100.0f, 0.0f,
                           0.0f, 100.0f};

float vaEstimateKF_K[4];

const float vaEstimateKF_H[4] = {1.0f, 0.0f,
                                 0.0f, 1.0f};

// extern vmc_leg_t right;
// extern vmc_leg_t left;

float vel_acc[2];
uint32_t OBSERVE_TIME = 3;
void Observe_task(void)
{
  while (INS.ins_flag == 0)
  {
    osDelay(1);
  }
  static float wr, wl = 0.0f;
  static float vrb, vlb = 0.0f;
  static float aver_v = 0.0f;

  xvEstimateKF_Init(&vaEstimateKF);

  while (1)
  {
		 wr = -chassis.Wheelmotor[1].speed_buck - INS.Gyro[0];
		 vrb = wr * Wheel_Radius;

		 wl = -chassis.Wheelmotor[0].speed_buck + INS.Gyro[0];
		 vlb = wl * Wheel_Radius;


		 aver_v = (vrb - vlb) / 2.0f; // ȡƽ��
		 xvEstimateKF_Update(&vaEstimateKF, INS.MotionAccel_n[1], aver_v);

		 chassis.v_filter.cur=vel_acc[0];
		 chassis.x_filter.cur=chassis.x_filter.cur+chassis.v_filter.cur*((float)OBSERVE_TIME/1000.0f);

//    chassis.v_filter.cur = (-chassis.Wheelmotor[1].speed_buck + chassis.Wheelmotor[0].speed_buck) * Wheel_Radius * 0.5f; // 0.0603�����Ӱ뾶������������ǽ��ٶȣ��˰뾶��õ����ٶȣ���ѧģ���ж����������˳ʱ��Ϊ��������Ҫ�˸�����
//    chassis.x_filter.cur = chassis.x_filter.cur + chassis.v_filter.cur * ((float)OBSERVE_TIME / 1000.0f);

    osDelay(OBSERVE_TIME);
  }
}

void xvEstimateKF_Init(KalmanFilter_t *EstimateKF)
{
  Kalman_Filter_Init(EstimateKF, 2, 0, 2); // ״̬����2ά û�п����� ��������2ά

  memcpy(EstimateKF->F_data, vaEstimateKF_F, sizeof(vaEstimateKF_F));
  memcpy(EstimateKF->P_data, vaEstimateKF_P, sizeof(vaEstimateKF_P));
  memcpy(EstimateKF->Q_data, vaEstimateKF_Q, sizeof(vaEstimateKF_Q));
  memcpy(EstimateKF->R_data, vaEstimateKF_R, sizeof(vaEstimateKF_R));
  memcpy(EstimateKF->H_data, vaEstimateKF_H, sizeof(vaEstimateKF_H));
}

void xvEstimateKF_Update(KalmanFilter_t *EstimateKF, float acc, float vel)
{
  EstimateKF->MeasuredVector[0] = vel;
  EstimateKF->MeasuredVector[1] = acc;

  Kalman_Filter_Update(EstimateKF);

  for (uint8_t i = 0; i < 2; i++)
  {
    vel_acc[i] = EstimateKF->FilteredValue[i];
  }
}

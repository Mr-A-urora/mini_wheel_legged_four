#ifndef __INS_TASK_H
#define __INS_TASK_H

#include "stdint.h"
#include "BMI088driver.h"
#include "QuaternionEKF.h"

#define X 0
#define Y 1
#define Z 2

#define INS_TASK_PERIOD 1

typedef struct
{
    float q[4]; // 四元数实部和虚部

    float Gyro[3];          // 陀螺仪角速度
    float Accel[3];         // 加速度计加速度
    float MotionAccel_b[3]; // 机体坐标系运动加速度
    float MotionAccel_n[3]; // 导航坐标系运动加速度
    float AccelLPF; // 加速度低通滤波系数

    // 加速度在惯性坐标系中的表示
    float xn[3];
    float yn[3];
    float zn[3];

    float atanxz;
    float atanyz;

    // 姿态角
    float Roll;
    float Pitch;
    float Yaw;
    float YawTotalAngle;
    float YawAngleLast;
    float YawRoundCount;

    float v_n; // 导航坐标系水平运动速度
    float x_n; // 导航坐标系水平运动位移

    uint8_t ins_flag; // INS初始化完成标志
} INS_t;

/**
 * @brief 惯性测量单元参数结构体，demo中可忽略
 *
 */
typedef struct
{
    uint8_t flag;

    float scale[3];

    float Yaw;
    float Pitch;
    float Roll;
} IMU_Param_t;

extern void INS_Init(void);
extern void INS_task(void);
extern INS_t INS;
void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q);
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q);

#endif

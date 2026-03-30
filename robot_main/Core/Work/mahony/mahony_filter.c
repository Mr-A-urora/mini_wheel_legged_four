#include "mahony_filter.h"

struct MAHONY_FILTER_t mahony_filter;

// 旋转矩阵更新，将地球坐标系 R 转换到机体坐标系 b 的旋转矩阵更新
void RotationMatrix_update(struct MAHONY_FILTER_t *mahony_filter)
{
    float q1q1 = mahony_filter->q1 * mahony_filter->q1;
    float q2q2 = mahony_filter->q2 * mahony_filter->q2;
    float q3q3 = mahony_filter->q3 * mahony_filter->q3;

    float q0q1 = mahony_filter->q0 * mahony_filter->q1;
    float q0q2 = mahony_filter->q0 * mahony_filter->q2;
    float q0q3 = mahony_filter->q0 * mahony_filter->q3;
    float q1q2 = mahony_filter->q1 * mahony_filter->q2;
    float q1q3 = mahony_filter->q1 * mahony_filter->q3;
    float q2q3 = mahony_filter->q2 * mahony_filter->q3;

    mahony_filter->rMat[0][0] = 1.0f - 2.0f * q2q2 - 2.0f * q3q3;
    mahony_filter->rMat[0][1] = 2.0f * (q1q2 - q0q3);
    mahony_filter->rMat[0][2] = 2.0f * (q1q3 + q0q2);

    mahony_filter->rMat[1][0] = 2.0f * (q1q2 + q0q3);
    mahony_filter->rMat[1][1] = 1.0f - 2.0f * q1q1 - 2.0f * q3q3;
    mahony_filter->rMat[1][2] = 2.0f * (q2q3 - q0q1);

    mahony_filter->rMat[2][0] = 2.0f * (q1q3 - q0q2);
    mahony_filter->rMat[2][1] = 2.0f * (q2q3 + q0q1);
    mahony_filter->rMat[2][2] = 1.0f - 2.0f * q1q1 - 2.0f * q2q2;
}

// 输入陀螺仪和加速度计的原始数据，存入滤波器结构体
void mahony_input(struct MAHONY_FILTER_t *mahony_filter, Axis3f gyro, Axis3f acc)
{
    mahony_filter->gyro = gyro;
    mahony_filter->acc = acc;
}

// 使用Mahony算法更新滤波器状态
void mahony_update(struct MAHONY_FILTER_t *mahony_filter)
{
    float normalise;
    float ex, ey, ez;

    // 加速度，单位转换
    // mahony_filter->gyro.x *= DEG2RAD;        // 转换为弧度
    // mahony_filter->gyro.y *= DEG2RAD;
    // mahony_filter->gyro.z *= DEG2RAD;;

    // 对加速度计测量值进行归一化，得到重力加速度的方向
    normalise = sqrt(mahony_filter->acc.x * mahony_filter->acc.x + mahony_filter->acc.y * mahony_filter->acc.y + mahony_filter->acc.z * mahony_filter->acc.z);

    mahony_filter->acc.x = mahony_filter->acc.x / normalise;
    mahony_filter->acc.y = mahony_filter->acc.y / normalise;
    mahony_filter->acc.z = mahony_filter->acc.z / normalise;

    // 加速度计读取的方向与重力加速度方向向量的叉积，计算误差向量
    // vx=mahony_filter->rMat[2][0]=2.0f * (q1q3 + -q0q2)
    // vy=mahony_filter->rMat[2][1]=2.0f * (q2q3 - -q0q1)
    // vz=mahony_filter->rMat[2][2]=1.0f - 2.0f * q1q1 - 2.0f * q2q2
    ex = (mahony_filter->acc.y * mahony_filter->rMat[2][2] - mahony_filter->acc.z * mahony_filter->rMat[2][1]);
    ey = (mahony_filter->acc.z * mahony_filter->rMat[2][0] - mahony_filter->acc.x * mahony_filter->rMat[2][2]);
    ez = (mahony_filter->acc.x * mahony_filter->rMat[2][1] - mahony_filter->acc.y * mahony_filter->rMat[2][0]);

    // 误差累积，实现积分补偿
    mahony_filter->exInt += mahony_filter->Ki * ex * mahony_filter->dt;
    mahony_filter->eyInt += mahony_filter->Ki * ey * mahony_filter->dt;
    mahony_filter->ezInt += mahony_filter->Ki * ez * mahony_filter->dt;

    // 用误差进行PI修正，补偿陀螺仪的漂移误差
    mahony_filter->gyro.x += mahony_filter->Kp * ex + mahony_filter->exInt;
    mahony_filter->gyro.y += mahony_filter->Kp * ey + mahony_filter->eyInt;
    mahony_filter->gyro.z += mahony_filter->Kp * ez + mahony_filter->ezInt;

    // 一阶龙格库塔法，四元数运动学方程的离散化形式和积分近似方法，更新四元数
    float q0Last = mahony_filter->q0;
    float q1Last = mahony_filter->q1;
    float q2Last = mahony_filter->q2;
    float q3Last = mahony_filter->q3;
    float halfT = mahony_filter->dt * 0.5f;
    mahony_filter->q0 += (-q1Last * mahony_filter->gyro.x - q2Last * mahony_filter->gyro.y - q3Last * mahony_filter->gyro.z) * halfT;
    mahony_filter->q1 += (q0Last * mahony_filter->gyro.x + q2Last * mahony_filter->gyro.z - q3Last * mahony_filter->gyro.y) * halfT;
    mahony_filter->q2 += (q0Last * mahony_filter->gyro.y - q1Last * mahony_filter->gyro.z + q3Last * mahony_filter->gyro.x) * halfT;
    mahony_filter->q3 += (q0Last * mahony_filter->gyro.z + q1Last * mahony_filter->gyro.y - q2Last * mahony_filter->gyro.x) * halfT;

    // 归一化四元数
    normalise = sqrt(mahony_filter->q0 * mahony_filter->q0 + mahony_filter->q1 * mahony_filter->q1 + mahony_filter->q2 * mahony_filter->q2 + mahony_filter->q3 * mahony_filter->q3);

    mahony_filter->q0 = mahony_filter->q0 / normalise;
    mahony_filter->q1 = mahony_filter->q1 / normalise;
    mahony_filter->q2 = mahony_filter->q2 / normalise;
    mahony_filter->q3 = mahony_filter->q3 / normalise;

    // 更新旋转矩阵
    mahony_filter->RotationMatrix_update(mahony_filter);
}

// 通过旋转矩阵获取姿态角
void mahony_output(struct MAHONY_FILTER_t *mahony_filter)
{
    // 计算roll pitch yaw 欧拉角
    mahony_filter->pitch = -asinf(mahony_filter->rMat[2][0]);
    mahony_filter->roll = atan2f(mahony_filter->rMat[2][1], mahony_filter->rMat[2][2]);
    mahony_filter->yaw = atan2f(mahony_filter->rMat[1][0], mahony_filter->rMat[0][0]);
}

// 初始化Mahony滤波器的参数和函数指针
void mahony_init(struct MAHONY_FILTER_t *mahony_filter, float Kp, float Ki, float dt)
{
    mahony_filter->Kp = Kp;
    mahony_filter->Ki = Ki;
    mahony_filter->dt = dt;
    mahony_filter->q0 = 1;
    mahony_filter->mahony_input = mahony_input;
    mahony_filter->mahony_update = mahony_update;
    mahony_filter->mahony_output = mahony_output;
    mahony_filter->RotationMatrix_update = RotationMatrix_update;
}

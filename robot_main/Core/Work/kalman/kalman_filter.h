#ifndef __KALMAN_FILTER_H
#define __KALMAN_FILTER_H

#include "arm_math.h"

#include "math.h"
#include "stdint.h"
#include "stdlib.h"

#ifndef user_malloc
#ifdef _CMSIS_OS_H
#define user_malloc pvPortMalloc
#else
#define user_malloc malloc
#endif
#endif

#define mat arm_matrix_instance_f32
#define Matrix_Init arm_mat_init_f32
#define Matrix_Add arm_mat_add_f32
#define Matrix_Subtract arm_mat_sub_f32
#define Matrix_Multiply arm_mat_mult_f32
#define Matrix_Transpose arm_mat_trans_f32
#define Matrix_Inverse arm_mat_inverse_f32

typedef struct kf_t
{
    float *FilteredValue; // 滤波后的值
    float *MeasuredVector; // 测量向量
    float *ControlVector; // 控制向量

    uint8_t xhatSize; // 状态向量维度
    uint8_t uSize; // 控制向量维度
    uint8_t zSize; // 测量向量维度

    uint8_t UseAutoAdjustment; // 是否启用自动调整
    uint8_t MeasurementValidNum; // 有效测量数量

    uint8_t *MeasurementMap;      // 测量与状态的关系 how measurement relates to the state
    float *MeasurementDegree;     // 测量值对应的H矩阵元素值 elements of each measurement in H
    float *MatR_DiagonalElements; // 测量方差 variance for each measurement
    float *StateMinVariance;      // 最小方差（协方差下界）抑制滤波器过度收敛 suppress filter excessive convergence
    uint8_t *temp;

    // 供用户自定义函数使用，作为标志位来判断是否需要跳过标准KF公式中的某一步
    uint8_t SkipEq1, SkipEq2, SkipEq3, SkipEq4, SkipEq5;

    // definiion of struct mat: rows & cols & pointer to vars
    mat xhat;          // x(k|k)
    mat xhatminus;     // x(k|k-1)
    mat u;             // control vector u
    mat z;             // measurement vector z
    mat P;             // covariance matrix P(k|k)
    mat Pminus;        // covariance matrix P(k|k-1)
    mat F, FT, temp_F; // state transition matrix F FT
    mat B;             // control matrix B
    mat H, HT;         // measurement matrix H
    mat Q;             // process noise covariance matrix Q
    mat R;             // measurement noise covariance matrix R
    mat K;             // kalman gain  K
    mat S, temp_matrix, temp_matrix1, temp_vector, temp_vector1;

    int8_t MatStatus;

    // 用户自定义函数，可用于替换或扩展标准KF的功能
    void (*User_Func0_f)(struct kf_t *kf);
    void (*User_Func1_f)(struct kf_t *kf);
    void (*User_Func2_f)(struct kf_t *kf);
    void (*User_Func3_f)(struct kf_t *kf);
    void (*User_Func4_f)(struct kf_t *kf);
    void (*User_Func5_f)(struct kf_t *kf);
    void (*User_Func6_f)(struct kf_t *kf);

    // 数据存储空间指针
    float *xhat_data, *xhatminus_data;
    float *u_data;
    float *z_data;
    float *P_data, *Pminus_data;
    float *F_data, *FT_data, *temp_F_data;
    float *B_data;
    float *H_data, *HT_data;
    float *Q_data;
    float *R_data;
    float *K_data;
    float *S_data, *temp_matrix_data, *temp_matrix_data1, *temp_vector_data, *temp_vector_data1;
} KalmanFilter_t;

extern uint16_t sizeof_float, sizeof_double;

void Kalman_Filter_Init(KalmanFilter_t *kf, uint8_t xhatSize, uint8_t uSize, uint8_t zSize);
void Kalman_Filter_Measure(KalmanFilter_t *kf);
void Kalman_Filter_xhatMinusUpdate(KalmanFilter_t *kf);
void Kalman_Filter_PminusUpdate(KalmanFilter_t *kf);
void Kalman_Filter_SetK(KalmanFilter_t *kf);
void Kalman_Filter_xhatUpdate(KalmanFilter_t *kf);
void Kalman_Filter_P_Update(KalmanFilter_t *kf);
float *Kalman_Filter_Update(KalmanFilter_t *kf);

#endif

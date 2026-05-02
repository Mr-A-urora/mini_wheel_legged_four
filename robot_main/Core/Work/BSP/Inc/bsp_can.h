#ifndef __BSP_CAN_H
#define __BSP_CAN_H

#include "main.h"

#define M_to_main_can 0x200
#define main_to_M_can 0x100


typedef enum eCanCmd
{
	CAN_CMD_MOTOR_DISABLE = 0,
	CAN_CMD_MOTOR_ENABLE,

	CAN_CMD_ERROR_REPORT,
	CAN_CMD_ERROR_RESET,

	CAN_CMD_SET_MODE,
	CAN_CMD_SET_POSITION,
	CAN_CMD_SET_VELOCITY,
	CAN_CMD_SET_CURRENT,
	CAN_CMD_NORMAL_REPORT,
	CAN_CMD_ENCODER_CHANGE,
	CAN_CMD_CANID_AUTOSET,
	CAN_CMD_ENCODER2_SET_ZERO,
	CAN_CMD_CAN_TIMEOUT_ENABLE,
} tCanCmd;

#define Can_Motor_Enable(X) M_to_main_can | (CAN_CMD_MOTOR_ENABLE << 4) | (X)
#define Can_Motor_report(X) M_to_main_can | (CAN_CMD_NORMAL_REPORT << 4) | (X)
#define Can_Motor_erro(X) M_to_main_can | (CAN_CMD_ERROR_REPORT << 4) | (X)

#define Can_Whell_Enable(X) M_to_main_can | (CAN_CMD_MOTOR_ENABLE << 4) | (X)
#define Can_Whell_report(X) M_to_main_can | (CAN_CMD_NORMAL_REPORT << 4) | (X)
#define Can_Whell_erro(X) M_to_main_can | (CAN_CMD_ERROR_REPORT << 4) | (X)




static inline float data_to_float(uint8_t *data)
{
    float tmp_float;
    *(((uint8_t *)(&tmp_float)) + 0) = data[0];
    *(((uint8_t *)(&tmp_float)) + 1) = data[1];
    *(((uint8_t *)(&tmp_float)) + 2) = data[2];
    *(((uint8_t *)(&tmp_float)) + 3) = data[3];
    return tmp_float;
}
static inline void int_to_data(int val, uint8_t *data)
{
    data[0] = *(((uint8_t *)(&val)) + 0);
    data[1] = *(((uint8_t *)(&val)) + 1);
    data[2] = *(((uint8_t *)(&val)) + 2);
    data[3] = *(((uint8_t *)(&val)) + 3);
}
static inline int data_to_int(uint8_t *data)
{
    int tmp_int;
    *(((uint8_t *)(&tmp_int)) + 0) = data[0];
    *(((uint8_t *)(&tmp_int)) + 1) = data[1];
    *(((uint8_t *)(&tmp_int)) + 2) = data[2];
    *(((uint8_t *)(&tmp_int)) + 3) = data[3];
    return tmp_int;
}
static inline void int16_to_data(int16_t val, uint8_t *data)
{
    data[0] = *(((uint8_t *)(&val)) + 0);
    data[1] = *(((uint8_t *)(&val)) + 1);
}
static inline int16_t data_to_int16(uint8_t *data)
{
    int16_t tmp_int;
    *(((uint8_t *)(&tmp_int)) + 0) = data[0];
    *(((uint8_t *)(&tmp_int)) + 1) = data[1];
    return tmp_int;
}
static inline void float_to_data(float val, uint8_t *data)
{
    data[0] = *(((uint8_t*)(&val)) + 0);
    data[1] = *(((uint8_t*)(&val)) + 1);
    data[2] = *(((uint8_t*)(&val)) + 2);
    data[3] = *(((uint8_t*)(&val)) + 3);
}
static inline void float_to_q69(float f, uint8_t *data)
{
    int16_t tmp_int16;
    // 检查输入是否超出 Q6.9 的范围
    if (f > 63.999969482421875f) // Q6.9 最大正数值（63.999969482421875）
        tmp_int16 = 0x7FF; // 最大值（16 位有符号整数的最大值是 32767，但这里 Q6.9 格式整数部分 6 位，所以最大值是 63）
    else if (f < -64.0f) // Q6.9 最小负数值（-64.0）
        tmp_int16 = 0x800; // 最小值（16 位有符号整数的最小值是 -32768，但这里 Q6.9 格式整数部分 6 位，所以最小值是 -64）
    else
        // 转换为 Q6.9 格式
        tmp_int16 = (int16_t)(f * 512.0f + 0.5f); // 加 0.5f 实现四舍五入，缩放因子为 512（2^9）

    data[0] = *(((uint8_t *)(&tmp_int16)) + 0);
    data[1] = *(((uint8_t *)(&tmp_int16)) + 1);
}

static inline float q69_to_float(uint8_t *data)
{
    int16_t tmp_int;
    *(((uint8_t *)(&tmp_int)) + 0) = data[0];
    *(((uint8_t *)(&tmp_int)) + 1) = data[1];
    return (float)tmp_int / 512.0f; // 缩放因子为 512（2^9）
}
static inline void float_to_q510(float f, uint8_t *data)
{
    int16_t tmp_int16;
    if (f > 31.9990234375f) {
        tmp_int16 = 0x7FFF; // 0x7FFF
    } else if (f < -32.0f) {
        tmp_int16 = 0x8000; // 0x8000
    } else {
        // 四舍五入并转换为Q5.10
        tmp_int16 = (int16_t)(f * 1024.0f + 0.5f);
    }

    // 小端字节序存储
    data[0] = (uint8_t)(tmp_int16 & 0xFF);       // 低字节
    data[1] = (uint8_t)((tmp_int16 >> 8) & 0xFF);// 高字节
}

static inline float q510_to_float(uint8_t *data)
{
    // 从小端字节序组合成int16_t
    int16_t tmp_int = (int16_t)((data[1] << 8) | data[0]);
    return (float)tmp_int / 1024.0f;
}


void FDCAN1_Config(void);
uint8_t FDCAN1_Send_Msg(uint32_t id, uint8_t *msg, uint32_t len);

#endif


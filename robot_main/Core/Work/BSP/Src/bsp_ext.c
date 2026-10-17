#include "bsp_ext.h"
#include "tim.h"
#include "cmsis_os.h"
#include "rc.h"
#include "xbox.h"
#include "chassis_task.h"

extern uint8_t start_flag;

#define ONE_PULSE (228)                           // 1 码计数个数
#define ZERO_PULSE (114)                          // 0 码计数个数
#define RESET_PULSE (80)                          // 80 复位电平个数（不能低于40）
#define LED_NUMS (13)                             // led 个数
#define LED_DATA_LEN (24)                         // led 长度，单个需要24个字节
#define WS2812_DATA_LEN (LED_NUMS * LED_DATA_LEN) // ws2812灯条需要的数组长度


static uint16_t RGB_buffur[RESET_PULSE + WS2812_DATA_LEN] = {0};
static uint16_t RGB_buffur_Copy[RESET_PULSE + WS2812_DATA_LEN] = {0};

void Ws2812_set_one_color(uint8_t R, uint8_t G, uint8_t B);

void Init_Beep(void)
{
    BEEP_ON();
    osDelay(50);
    BEEP_OFF();
    osDelay(50);
    BEEP_ON();
    osDelay(50);
    BEEP_OFF();
    osDelay(50);
    BEEP_ON();
    osDelay(100);
    BEEP_OFF();
}

/* TIM2 clock: 275 MHz, PSC=2749, ARR=1999; one CCR count = 10 us.
 * Array order: CH2/right lift, CH3/left reach, CH4/gripper.
 * Joint limits remain narrow until the full mechanical travel is measured.
 */
#define SERVO_LIFT_STEP 6
#define SERVO_REACH_STEP 6
#define SERVO_CLAW_CLOSE_STEP 3
#define SERVO_CLAW_OPEN_STEP 12
volatile uint16_t servo_p[3] = {153, 144, 150}; /* PWM command counts, visible in Watch */
static const uint16_t servo_min[3] = {123, 100, 150};
static const uint16_t servo_max[3] = {175, 172, 195};
static const uint32_t servo_channels[3] = {TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4};
static uint32_t servo_last_tick;
static uint8_t servo_initialized;

void Init_Servo(void)
{
    uint8_t i;
    servo_initialized = 0;
    for (i = 0; i < 3; i++) {
        HAL_TIM_PWM_Stop(&htim2, servo_channels[i]);
    }
    servo_p[0] = 153;
    servo_p[1] = 144;
    servo_p[2] = 150;
    for (i = 0; i < 3; i++) {
        __HAL_TIM_SET_COMPARE(&htim2, servo_channels[i], servo_p[i]);
    }
    for (i = 0; i < 3; i++) {
        if (HAL_TIM_PWM_Start(&htim2, servo_channels[i]) != HAL_OK) {
            uint8_t j;
            for (j = 0; j < 3; j++) {
                HAL_TIM_PWM_Stop(&htim2, servo_channels[j]);
            }
            Error_Handler();
            return;
        }
    }
    servo_last_tick = HAL_GetTick();
    servo_initialized = 1;
}

void Init_Ws2812(void)
{
    Ws2812_set_one_color(0x0f, 0x00, 0);
}

void Servo_task(void)
{
    uint8_t i;
    uint8_t close = xbox.btnRB != 0;
    uint8_t open = xbox.btnLB != 0;
    uint32_t now = HAL_GetTick();
    int32_t vertical, reach;
    int32_t next[3];

    if (!servo_initialized || (uint32_t)(now - servo_last_tick) < 100U) {
        return;
    }
    servo_last_tick = now;
    /* Keep PWM active at the current pose while chassis control is disabled. */
    if (start_flag == 0) { return; }
    vertical = (xbox.btnDirUp != 0) - (xbox.btnDirDown != 0);
    reach = (xbox.btnDirRight != 0) - (xbox.btnDirLeft != 0);
    /* Diagonal D-pad input pauses the two arm joints. */
    if (vertical && reach) { vertical = 0; reach = 0; }
    next[0] = (int32_t)servo_p[0] - vertical * SERVO_LIFT_STEP; /* up = right joint clockwise */
    next[1] = (int32_t)servo_p[1] - reach * SERVO_REACH_STEP;    /* forward = left joint clockwise */
    next[2] = servo_p[2];
    /* Neither/both shoulders: hold. Close slowly; open quickly. */
    if (close != open) {
        next[2] += close ? SERVO_CLAW_CLOSE_STEP : -SERVO_CLAW_OPEN_STEP;
    }
    for (i = 0; i < 3; i++) {
        if (next[i] < servo_min[i]) { next[i] = servo_min[i]; }
        if (next[i] > servo_max[i]) { next[i] = servo_max[i]; }
        servo_p[i] = (uint16_t)next[i];
        __HAL_TIM_SET_COMPARE(&htim2, servo_channels[i], servo_p[i]);
    }
}

void ws2812_set_RGB(uint8_t R, uint8_t G, uint8_t B, uint16_t num)
{
    // 指针偏移:需要跳过复位信号的N个0
    uint16_t *p = (RGB_buffur + RESET_PULSE) + (num * LED_DATA_LEN);

    for (uint16_t i = 0; i < 8; i++)
    {
        // 填充数组
        p[i] = (G << i) & (0x80) ? ONE_PULSE : ZERO_PULSE;
        p[i + 8] = (R << i) & (0x80) ? ONE_PULSE : ZERO_PULSE;
        p[i + 16] = (B << i) & (0x80) ? ONE_PULSE : ZERO_PULSE;
    }
}

void Ws2812_set_one_color(uint8_t R, uint8_t G, uint8_t B)
{
    for (uint8_t i = 0; i < LED_NUMS; i++)
    {
        ws2812_set_RGB(R, G, B, i);
    }

    memcpy(RGB_buffur_Copy, RGB_buffur, sizeof(RGB_buffur));
    HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, (uint32_t *)RGB_buffur_Copy, RESET_PULSE + WS2812_DATA_LEN);
    HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_2, (uint32_t *)RGB_buffur, RESET_PULSE + WS2812_DATA_LEN);
}

void Ws2812_task(void)
{
    static uint8_t cnt = 0;
    cnt++;
    if (cnt == 5){
        Ws2812_set_one_color(10, 0x00, 0);
    }
    else if (cnt == 10){
        cnt = 0;
        Ws2812_set_one_color(0, 10, 0);
    }
}

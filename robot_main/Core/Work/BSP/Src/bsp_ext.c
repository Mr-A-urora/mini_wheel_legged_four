#include "bsp_ext.h"
#include "tim.h"
#include "cmsis_os.h"
#include "rc.h"

#define ONE_PULSE (228)                           // 1 码计数个数
#define ZERO_PULSE (114)                          // 0 码计数个数
#define RESET_PULSE (80)                          // 80 复位电平个数（不能低于40）
#define LED_NUMS (13)                             // led 个数
#define LED_DATA_LEN (24)                         // led 长度，单个需要24个字节
#define WS2812_DATA_LEN (LED_NUMS * LED_DATA_LEN) // ws2812灯条需要的数组长度

uint8_t servo_p[2]={51,240};
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

void Init_Servo(void)
{
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
		TIM2->CCR3=servo_p[0];
		TIM2->CCR4=servo_p[1];
}

void Init_Ws2812(void)
{
    Ws2812_set_one_color(0x0f, 0x00, 0);
}

void Servo_task(void)
{		
    static uint8_t cnt=0;
    if(rc.wheel!=0){
        if(cnt<2){
            cnt++;
            servo_p[0]=51;
            servo_p[1]=240-30;	
            TIM2->CCR3=servo_p[0];
            TIM2->CCR4=servo_p[1];
        }
        else if(cnt<4){
            cnt++;
            servo_p[0]=51+30;
            servo_p[1]=240;	
            TIM2->CCR3=servo_p[0];
            TIM2->CCR4=servo_p[1];
        }
        else{
            cnt=0;
        }		
    }
    else{
        cnt=0;
        servo_p[0]=51;
        servo_p[1]=240;	
        TIM2->CCR3=servo_p[0];
        TIM2->CCR4=servo_p[1];
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

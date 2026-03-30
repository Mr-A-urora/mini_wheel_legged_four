#ifndef __BSP_EXT_H
#define __BSP_EXT_H

#include "main.h"

#define BEEP_ON() TIM15->CCR1=800
#define BEEP_OFF() TIM15->CCR1=0

void Init_Beep(void);
void Init_Servo(void);
void Init_Ws2812(void);
void Servo_task(void);
void Ws2812_task(void);

#endif

/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ins_task.h"
#include "usbd_cdc_if.h"
#include "string.h"
#include "chassis_task.h"
#include "bsp_ext.h"
#include "bsp_can.h"
#include "observe_task.h"
#include "vmc.h"
#include "DEFS_Parser.h"
#include "usart.h"
#include "vmc.h"
#include "adc.h"
#include "spi.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
extern pid_type_def TURN_PID;
extern pid_type_def Roll_Contorl_PID;
extern float vmc_pid_param[3];
extern float K_Matrix[4];
extern float yaw_init;
uint8_t usb_tx_buf[512];
uint8_t usb_rx_buf[512];
uint8_t usb_rx_flag;
float vbus;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for myTask_imu */
osThreadId_t myTask_imuHandle;
const osThreadAttr_t myTask_imu_attributes = {
    .name = "myTask_imu",
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityLow,
};
/* Definitions for myTask_Chassis */
osThreadId_t myTask_ChassisHandle;
const osThreadAttr_t myTask_Chassis_attributes = {
    .name = "myTask_Chassis",
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityLow,
};
/* Definitions for myTask_LED */
osThreadId_t myTask_LEDHandle;
const osThreadAttr_t myTask_LED_attributes = {
    .name = "myTask_LED",
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityLow,
};
/* Definitions for myTask_observe */
osThreadId_t myTask_observeHandle;
const osThreadAttr_t myTask_observe_attributes = {
    .name = "myTask_observe",
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityLow,
};
/* Definitions for myTask_vmc */
osThreadId_t myTask_vmcHandle;
const osThreadAttr_t myTask_vmc_attributes = {
    .name = "myTask_vmc",
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityLow,
};
/* Definitions for myTask_adc */
osThreadId_t myTask_adcHandle;
const osThreadAttr_t myTask_adc_attributes = {
    .name = "myTask_adc",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityLow,
};
/* Definitions for myTask_motorErr */
osThreadId_t myTask_motorErrHandle;
const osThreadAttr_t myTask_motorErr_attributes = {
    .name = "myTask_motorErr",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityLow,
};
/* Definitions for myTask_usb */
osThreadId_t myTask_usbHandle;
const osThreadAttr_t myTask_usb_attributes = {
    .name = "myTask_usb",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityLow,
};
/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */
void StartDefaultTask(void *argument);
void StartTask_imu(void *argument);
void StartTask_Chassis(void *argument);
void StartTask_LED(void *argument);
void StartTask_observe(void *argument);
void StartTask_vmc(void *argument);
void StartTask_adc(void *argument);
void StartTask_motorError(void *argument);
void StartTask_usb(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void)
{
  /* USER CODE BEGIN Init */
  FDCAN1_Config();
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of myTask_imu */
  myTask_imuHandle = osThreadNew(StartTask_imu, NULL, &myTask_imu_attributes);

  /* creation of myTask_Chassis */
  myTask_ChassisHandle = osThreadNew(StartTask_Chassis, NULL, &myTask_Chassis_attributes);

  /* creation of myTask_LED */
  myTask_LEDHandle = osThreadNew(StartTask_LED, NULL, &myTask_LED_attributes);

  /* creation of myTask_observe */
  myTask_observeHandle = osThreadNew(StartTask_observe, NULL, &myTask_observe_attributes);

  /* creation of myTask_vmc */
  myTask_vmcHandle = osThreadNew(StartTask_vmc, NULL, &myTask_vmc_attributes);

  /* creation of myTask_adc */
  myTask_adcHandle = osThreadNew(StartTask_adc, NULL, &myTask_adc_attributes);

  /* creation of myTask_motorErr */
  myTask_motorErrHandle = osThreadNew(StartTask_motorError, NULL, &myTask_motorErr_attributes);

  /* creation of myTask_usb */
  myTask_usbHandle = osThreadNew(StartTask_usb, NULL, &myTask_usb_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  INIT_PARSER(parser, NULL);
  parser.Init(&parser, '$', 0);
  /* Infinite loop */
  for (;;)
  {
    if (uart1_rx_ok_flag)
    {
      uart1_rx_ok_flag = 0;
      printf("\r\n\r\n-- [DEFS] new frame --\r\n");
      // $SET /test/p 0.009A*/\r\n
      for (int j = 0; j < uart1_rx_len; j++)
      {
        if (parser.PushChar2RawBuf(&parser, uart1_rx_buf[j]))
        {
          if (parser.Verify(&parser))
          {
            if (parser.Parse(&parser))
            {
              parser.parsed_value = atof(parser.payload.value);
              // MatchThenSet_FLOAT(&parser, "/data/1", &joint_angle, NULL);
              MatchThenSet_FLOAT(&parser, "/pitch/0", &chassis.pitch.tar, NULL);
              MatchThenSet_FLOAT(&parser, "/K_Matrix/0", &K_Matrix[0], NULL);
              MatchThenSet_FLOAT(&parser, "/K_Matrix/1", &K_Matrix[1], NULL);
              MatchThenSet_FLOAT(&parser, "/K_Matrix/2", &K_Matrix[2], NULL);
              MatchThenSet_FLOAT(&parser, "/K_Matrix/3", &K_Matrix[3], NULL);
              MatchThenSet_FLOAT(&parser, "/TURN_PID/p", &TURN_PID.Kp, NULL);
              MatchThenSet_FLOAT(&parser, "/TURN_PID/d", &TURN_PID.Kd, NULL);
              MatchThenSet_FLOAT(&parser, "/Roll_PID/p", &Roll_Contorl_PID.Kp, NULL);
              MatchThenSet_FLOAT(&parser, "/Roll_PID/i", &Roll_Contorl_PID.Ki, NULL);
              MatchThenSet_FLOAT(&parser, "/Roll_PID/d", &Roll_Contorl_PID.Kd, NULL);
              MatchThenSet_FLOAT(&parser, "/vmc_pid/p", &vmc_pid_param[0], NULL);
              MatchThenSet_FLOAT(&parser, "/vmc_pid/i", &vmc_pid_param[1], NULL);
              MatchThenSet_FLOAT(&parser, "/vmc_pid/d", &vmc_pid_param[2], NULL);
            }
          }
          parser.Reset(&parser);
        }
      }
      HAL_UART_Receive_DMA(&huart1, uart1_rx_buf, 100);
    }

    printf("%.1f\n%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n", vbus, robot[0].d_virtual_leg_length, robot[1].d_virtual_leg_length, robot[0].virtual_leg_length_back, robot[1].virtual_leg_length_back, robot[0].virtual_support_force_back, robot[1].virtual_support_force_back);
    // printf("%.3f,%.3f,%.3f,%.3f\n",K_Matrix[0],K_Matrix[1],K_Matrix[2],K_Matrix[3]);
    // printf("%.2f,%.2f\n",INS.Pitch,robot[0].virtual_leg_length_back);
    // printf("%.2f,%.2f,%.2f,%.2f,%.2f\n",vbus,INS.Roll,VMC_PID[0].out,VMC_PID[1].out,Roll_Contorl_PID.out);
    // printf("%.3f,%.3f,%.5f,%.5f,%.3f,%.3f\n",chassis.pitch.err,chassis.d_pitch.err,chassis.x_filter.err,chassis.v_filter.err,robot[0].virtual_support_force_back,robot[1].virtual_support_force_back);
    // printf("%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",TURN_PID.out,yaw_init,INS.YawTotalAngle,INS.Gyro[2],TURN_PID.Kp,TURN_PID.Kd);
    // printf("%.3f,%.3f",chassis.Jointmotor[0].iq_set,INS.Roll);

    // printf("%.3f,%.3f,%.3f,%.3f\n",robot[0].virtual_support_force_back,robot[1].virtual_support_force_back,robot[0].JointForce_Virtual2Real_K,robot[1].JointForce_Virtual2Real_K);
    osDelay(100);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask_imu */
/**
 * @brief Function implementing the myTask_imu thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask_imu */
void StartTask_imu(void *argument)
{
  /* USER CODE BEGIN StartTask_imu */

  /* Infinite loop */
  for (;;)
  {
    INS_task();
  }
  /* USER CODE END StartTask_imu */
}

/* USER CODE BEGIN Header_StartTask_Chassis */
/**
 * @brief Function implementing the myTask_Chassis thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask_Chassis */
void StartTask_Chassis(void *argument)
{
  /* USER CODE BEGIN StartTask_Chassis */
  /* Infinite loop */
  for (;;)
  {
    Chassis_task();
  }
  /* USER CODE END StartTask_Chassis */
}

/* USER CODE BEGIN Header_StartTask_LED */
/**
 * @brief Function implementing the myTask_LED thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask_LED */
void StartTask_LED(void *argument)
{
  /* USER CODE BEGIN StartTask_LED */
  Init_Servo();
  Init_Ws2812();
  osDelay(500);
  /* Infinite loop */
  for (;;)
  {
    Servo_task();
    Ws2812_task();
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    osDelay(100);
  }
  /* USER CODE END StartTask_LED */
}

/* USER CODE BEGIN Header_StartTask_observe */
/**
 * @brief Function implementing the myTask_observe thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask_observe */
void StartTask_observe(void *argument)
{
  /* USER CODE BEGIN StartTask_observe */
  /* Infinite loop */
  for (;;)
  {
    Observe_task();
  }
  /* USER CODE END StartTask_observe */
}

/* USER CODE BEGIN Header_StartTask_vmc */
/**
 * @brief Function implementing the myTask_vmc thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask_vmc */
void StartTask_vmc(void *argument)
{
  /* USER CODE BEGIN StartTask_vmc */
  /* Infinite loop */
  for (;;)
  {
    vmc_task();
  }
  /* USER CODE END StartTask_vmc */
}

/* USER CODE BEGIN Header_StartTask_adc */
/**
 * @brief Function implementing the myTask_adc thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask_adc */
void StartTask_adc(void *argument)
{
  /* USER CODE BEGIN StartTask_adc */
  uint32_t Vbus_adc;
  /* Infinite loop */
  for (;;)
  {
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 8); //??????
    Vbus_adc = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
    vbus = Vbus_adc * 3.3 / 65536 * 11;
    osDelay(500);
  }
  /* USER CODE END StartTask_adc */
}

/* USER CODE BEGIN Header_StartTask_motorError */
/**
 * @brief Function implementing the myTask_motorErr thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask_motorError */
void StartTask_motorError(void *argument)
{
  /* USER CODE BEGIN StartTask_motorError */
  /* Infinite loop */
  for (;;)
  {
    if (chassis.Jointmotor[0].error_status == 1)
    {
      printf("Jointmotor 0 error code: %x\r\n", chassis.Jointmotor[0].error_code);
      osDelay(500);
    }
    if (chassis.Jointmotor[1].error_status == 1)
    {
      printf("Jointmotor 1 error code: %x\r\n", chassis.Jointmotor[1].error_code);
      osDelay(500);
    }
    if (chassis.Wheelmotor[0].error_status == 1)
    {
      printf("Wheelmotor 0 error code: %x\r\n", chassis.Wheelmotor[0].error_code);
      osDelay(500);
    }
    if (chassis.Wheelmotor[1].error_status == 1)
    {
      printf("Wheelmotor 1 error code: %x\r\n", chassis.Wheelmotor[1].error_code);
      osDelay(500);
    }
    osDelay(1);
  }
  /* USER CODE END StartTask_motorError */
}

/* USER CODE BEGIN Header_StartTask_usb */
/**
 * @brief Function implementing the myTask_usb thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask_usb */
void StartTask_usb(void *argument)
{
  uint16_t len;
  char imu_check[10] = {"imu set"};
  /* USER CODE BEGIN StartTask_usb */
  /* Infinite loop */
  for (;;)
  {
    if (usb_rx_flag)
    {
      usb_rx_flag = 0;
      if (usb_rx_buf[0] == 7 && memcmp(imu_check, usb_rx_buf + 1, 7) == 0)
      {
        vTaskSuspend(myTask_imuHandle);
        len = sprintf((char *)usb_tx_buf, "start imu check\n");
        CDC_Transmit_HS(usb_tx_buf, len);
        osDelay(100);
        while (BMI088_init(&hspi1, 1) != BMI088_NO_ERROR)
        {
          ;
        }
        len = sprintf((char *)usb_tx_buf, "BMI088_Offset-->>\nGyro:\n%.12f\n%.12f\n%.12f\nAccel:\n%.12f\n%.12f\n%.12f\nPlease save and reboot", BMI088.GyroOffset[0], BMI088.GyroOffset[1], BMI088.GyroOffset[2], BMI088.AccelOffset[0], BMI088.AccelOffset[1], BMI088.AccelOffset[2]);
        CDC_Transmit_HS(usb_tx_buf, len);
      }
    }
    else
    {
      len = sprintf((char *)usb_tx_buf, "INS: %.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n",
                    INS.Accel[0], INS.Accel[1], INS.Accel[2],
                    INS.Gyro[0], INS.Gyro[1], INS.Gyro[2]);
      CDC_Transmit_HS(usb_tx_buf, len);
    }

    osDelay(500);
  }
  /* USER CODE END StartTask_usb */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

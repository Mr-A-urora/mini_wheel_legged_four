#include "bsp_can.h"
#include "fdcan.h"
#include "bsp_motor.h"
#include "chassis_task.h"

void FDCAN1_Config(void)
{
  FDCAN_FilterTypeDef sFilterConfig;
  /* Configure Rx filter */
  sFilterConfig.IdType = FDCAN_STANDARD_ID;
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterType = FDCAN_FILTER_MASK;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1 = 0x0000;
  sFilterConfig.FilterID2 = 0x0000;

  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  //  sFilterConfig.IdType = FDCAN_STANDARD_ID;
  //  sFilterConfig.FilterIndex = 0;
  //  sFilterConfig.FilterType = FDCAN_FILTER_MASK;
  //  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  //  sFilterConfig.FilterID1 = 0x0000020A;
  //  sFilterConfig.FilterID2 = 0x1FFFFFFF;

  //  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
  //  {
  //    Error_Handler();
  //  }

  /* Configure global filter:
     Filter all remote frames with STD and EXT ID
     Reject non matching frames with STD ID and EXT ID */
  //  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
  //  {
  //    Error_Handler();
  //  }

  /* Start the FDCAN module */
  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
  {
    Error_Handler();
  }
}

FDCAN_TxHeaderTypeDef TxHeader;

uint8_t FDCAN1_Send_Msg(uint32_t id, uint8_t *msg, uint32_t len)
{
  TxHeader.Identifier = id;                // 32位ID
  TxHeader.IdType = FDCAN_STANDARD_ID;     // 标准ID
  TxHeader.TxFrameType = FDCAN_DATA_FRAME; // 数据�?????
  TxHeader.DataLength = len;               // 数据长度
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;           // 关闭速率切换
  TxHeader.FDFormat = FDCAN_CLASSIC_CAN;            // 传统的CAN模式
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS; // 无发送事�?????
  TxHeader.MessageMarker = 0;

  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, msg) != HAL_OK)
    return 1; // 发�??
  return 0;
}

int erro;
FDCAN_RxHeaderTypeDef RxHeader;
uint8_t CAN1_RxData[8];
static uint8_t tim;

uint32_t cnt1, cnt2;
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{

  switch (RxHeader.Identifier)
  {
  case Can_Motor_report(Joint_Motor_Left_ID):
    Motorparam_updata(&chassis.Jointmotor[0], CAN1_RxData, 8);
    break;
  case Can_Motor_report(Joint_Motor_Right_ID):
    Motorparam_updata(&chassis.Jointmotor[1], CAN1_RxData, 8);
    break;
  case Can_Whell_report(Wheel_Motor_Left_ID):
    Motorparam_updata(&chassis.Wheelmotor[0], CAN1_RxData, 8);
    break;
  case Can_Whell_report(Wheel_Motor_Right_ID):
    Motorparam_updata(&chassis.Wheelmotor[1], CAN1_RxData, 8);
    break;
  case Can_Motor_erro(Joint_Motor_Left_ID):
    chassis.Jointmotor[0].error_code = data_to_int(CAN1_RxData);
    chassis.Jointmotor[0].error_status = 1;
    break;
  case Can_Motor_erro(Joint_Motor_Right_ID):
    chassis.Jointmotor[1].error_code = data_to_int(CAN1_RxData);
    chassis.Jointmotor[1].error_status = 1;
    break;
  case Can_Whell_erro(Wheel_Motor_Left_ID):
    chassis.Wheelmotor[0].error_code = data_to_int(CAN1_RxData);  
    chassis.Wheelmotor[0].error_status = 1;
    break;
  case Can_Whell_erro(Wheel_Motor_Right_ID):
    chassis.Wheelmotor[1].error_code = data_to_int(CAN1_RxData);
    chassis.Wheelmotor[1].error_status = 1;
    break;
  default:
    break;
  }
  if (++tim > 200)
  {
    tim = 0;
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_14);
  }
  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
  {
    /* Retrieve Rx messages from RX FIFO0 */
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, CAN1_RxData) != HAL_OK)
    {
      Error_Handler();
    }
  }
}

/**
  ******************************************************************************
  * @file    app_master.c
  * @author  MCD Application Team
  * @brief   Entry point AT master application
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/

#include "master_app.h"
#include "stm32_lpm.h"
#include "sys_app.h"
#include "lora_driver.h"
/*#include "stm32_seq.h"*/  /* if using sequencer uncomment */
#include "usart.h"
#include "app_master.h"
#include "sys_conf.h"
#include <stdio.h>
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */



/* External variables ---------------------------------------------------------*/
/* USER CODE BEGIN EV */
extern RTC_HandleTypeDef hrtc;
/* USER CODE END EV */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
//UART_WakeUpTypeDef WakeUpSelection;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Exported functions --------------------------------------------------------*/


void MX_Master_Init(void)
{
  /* USER CODE BEGIN MX_LoRaWAN_Init_1 */

  /* USER CODE END MX_LoRaWAN_Init_1 */
  SystemApp_Init();
  /* USER CODE BEGIN MX_LoRaWAN_Init_2 */

  /* USER CODE END MX_LoRaWAN_Init_2 */


  MasterApp_Init();
  /* USER CODE BEGIN MX_LoRaWAN_Init_3 */

  /* USER CODE END MX_LoRaWAN_Init_3 */
}

void MX_Master_Process(void)
{

    /*if using sequencer comment the following code*/
    /* run the LoRa Modem state machine*/
    Lora_fsm();
    DISABLE_IRQ();
    /* if an interrupt has occurred after DISABLE_IRQ, it is kept pending
     * and cortex will not enter low power anyway  */
    if ((lora_getDeviceState() == DEVICE_SLEEP && lora_getDeviceSubState() != DEVICE_JOIN_ON_GOING) && (HW_UART_Modem_IsNewCharReceived() == RESET))
    {

#if defined (LOW_POWER_DISABLE) && (LOW_POWER_DISABLE == 0)
     // DBG_PRINTF("Entering low power mode");
      //HAL_PWREx_EnableInternalWakeUpLine();
      HAL_RTCEx_SetWakeUpTimer_IT(&hrtc,10,RTC_WAKEUPCLOCK_CK_SPRE_16BITS); //3600
     /* WakeUpSelection.WakeUpEvent = UART_WAKEUP_ON_READDATA_NONEMPTY;
      if (HAL_UARTEx_StopModeWakeUpSourceConfig(&hlpuart1, WakeUpSelection)!= HAL_OK)
      {
      		Error_Handler();
      }*/
      UTIL_LPM_EnterLowPower();
      //DBG_PRINTF("Exiting low power mode");
      HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
#elif !defined (LOW_POWER_DISABLE)
#endif
    }
    ENABLE_IRQ();
  /* USER CODE BEGIN MX_LoRaWAN_Process_1 */

  /* USER CODE END MX_LoRaWAN_Process_1 */
  /* if using sequencer uncomment the call to the sequencer*/
  /*UTIL_SEQ_Run(UTIL_SEQ_DEFAULT);*/
  /* USER CODE BEGIN MX_LoRaWAN_Process_2 */

  /* USER CODE END MX_LoRaWAN_Process_2 */
}

/* USER CODE BEGIN EF */

/* USER CODE END EF */

/* Private Functions Definition -----------------------------------------------*/
/* USER CODE BEGIN PrFD */

/* USER CODE END PrFD */

/**
  * @brief  Configures LED GPIO for external modem.
  * @param  Led: Led to be configured.
  * @retval None
  */
void Master_LED_Modem_Init()
{

}



/**
  * @brief  DeInit LEDs GPIO for external modem.
  * @param  Led: LED to be de-init.
  * @note Led DeInit does not disable the GPIO clock nor disable the Mfx
  * @retval None
  */
void Master_LED_Modem_DeInit()
{

}


/**
  * @brief  Turns selected LED Modem On.
  * @param  Led: Specifies the Led to be set on.
  * @retval None
  */
void Master_LED_Modem_On()
{

}

/**
  * @brief  Turns selected LED Modem Off.
  * @param  Led: Specifies the Led to be set off.
  * @retval None
  */
void Master_LED_Modem_Off()
{

}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

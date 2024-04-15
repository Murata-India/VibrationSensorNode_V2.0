/**
  ******************************************************************************
  * @file    sys_app.c
  * @author  MCD Application Team
  * @brief   Initializes HW and SW system entities (not related to the radio)
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
#include <adc_if.h>
#include <platform.h>
#include <rtc_if.h>
#include <stdio.h>
#include <sys_app.h>
#include <sys_debug.h>
#include <utilities_def.h>
#include "adc_if.h"
/*#include "stm32_seq.h"*/
#include "stm32_systime.h"
#include "stm32_lpm.h"
#include "../../App/Inc/app_master.h"
#if USE_LRWAN_NS1
#include "usart.h"
#endif

#include"../../App/Inc/atcmd.h"       /* preprocessing definition in sys_conf.h*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* External variables ---------------------------------------------------------*/
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
#define MAX_TS_SIZE (int) 16

/**
  * Defines the maximum battery level
  */
#define LORAWAN_MAX_BAT   254
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

/* Exported functions ---------------------------------------------------------*/
/**
  * @brief initialises the system (dbg pins, trace, mbmux, systiemr, LPM, ...)
  * @param none
  * @retval  none
  */
void SystemApp_Init(void)
{
  /* USER CODE BEGIN SystemApp_Init_1 */

  /* USER CODE END SystemApp_Init_1 */

  /*Initialises timer and RTC*/
  UTIL_TIMER_Init();

// Gpio_PreInit();

  /* Configure the debug mode*/
  DBG_Init();

  /*Initialize the terminal */
  //UTIL_ADV_TRACE_Init();

  /*Set verbose LEVEL*/
  /*UTIL_ADV_TRACE_SetVerboseLevel(VERBOSE_LEVEL);*/
  /*Initialize the temperature and Battery measurement services */
  //SYS_InitMeasurement();


  /*Init low power manager*/
  UTIL_LPM_Init();
  /* Disable Stand-by mode */
  UTIL_LPM_SetOffMode((1 << CFG_LPM_APPLI_Id), UTIL_LPM_DISABLE);

#if defined (LOW_POWER_DISABLE) && (LOW_POWER_DISABLE == 1)
  /* Disable Stop Mode */
  UTIL_LPM_SetStopMode((1 << CFG_LPM_APPLI_Id), UTIL_LPM_DISABLE);
#elif !defined (LOW_POWER_DISABLE)
#error LOW_POWER_DISABLE not defined
#endif /* LOW_POWER_DISABLE */
  /* USER CODE BEGIN SystemApp_Init_2 */

  /* USER CODE END SystemApp_Init_2 */
}


/**
  * @brief This function Initializes the hardware Ios
  * @param None
  * @retval None
  */
void HW_IoInit(void)
{
    MX_GPIO_Init();
   // HAL_UART_MspInit(&hlpuart1);
    HAL_TIM_Base_MspInit(&htim1);
    //HAL_ADC_MspInit(&hadc1);
    HAL_UART_MspInit(&huart1);
}

/**
  * @brief This function Deinitializes the hardware Ios
  * @param None
  * @retval None
  */
void HW_IoDeInit(void)
{
  HAL_GPIO_DeInit(VBUS_DETECT_GPIO_Port, VBUS_DETECT_Pin);
  HAL_GPIO_DeInit(SENSOR_PWR_CNTRL_GPIO_Port,SENSOR_PWR_CNTRL_Pin);
  HAL_GPIO_DeInit(LED_CNTRL_GPIO_Port,LED_CNTRL_Pin);
  //Modem_IO_DeInit();
  //HAL_ADC_MspDeInit(&hadc1); //DMA is disabled by this call.
  HAL_TIM_Base_MspDeInit(&htim1);
  HAL_UART_MspDeInit(&huart1);
  //Gpio_PreInit();
}
/**
  * @brief redefines __weak function in stm32_seq.c such to enter low power
  * @param none
  * @retval  none
  */
void UTIL_SEQ_Idle(void)
{
  /* USER CODE BEGIN UTIL_SEQ_Idle_1 */

  /* USER CODE END UTIL_SEQ_Idle_1 */
  UTIL_LPM_EnterLowPower();
  /* USER CODE BEGIN UTIL_SEQ_Idle_2 */

  /* USER CODE END UTIL_SEQ_Idle_2 */
}

uint8_t GetBatteryLevel(void)
{
  uint8_t batteryLevel = 0;
  uint16_t batteryLevelmV;

  /* USER CODE BEGIN GetBatteryLevel_0 */

  /* USER CODE END GetBatteryLevel_0 */

  batteryLevelmV = (uint16_t) SYS_GetBatteryLevel();

  /* Convert batterey level from mV to linea scale: 1 (very low) to 254 (fully charged) */
  if (batteryLevelmV > VDD_BAT)
  {
    batteryLevel = LORAWAN_MAX_BAT;
  }
  else if (batteryLevelmV < VDD_MIN)
  {
    batteryLevel = 0;
  }
  else
  {
    batteryLevel = (((uint32_t)(batteryLevelmV - VDD_MIN) * LORAWAN_MAX_BAT) / (VDD_BAT - VDD_MIN));
  }

  /* USER CODE BEGIN GetBatteryLevel_2 */

  /* USER CODE END GetBatteryLevel_2 */

  return batteryLevel;  /* 1 (very low) to 254 (fully charged) */
}

float GetTemperatureLevel(void)
{
  float temperatureLevel = 0;

  temperatureLevel = SYS_GetTemperatureLevel();
  /* USER CODE BEGIN GetTemperatureLevel */

  /* USER CODE END GetTemperatureLevel */
  return temperatureLevel;
}

void GetUniqueId(uint8_t *id)
{
  /* USER CODE BEGIN GetUniqueId_1 */

  /* USER CODE END GetUniqueId_1 */
  uint32_t ID_1_3_val = HAL_GetUIDw0() + HAL_GetUIDw2();
  uint32_t ID_2_val = HAL_GetUIDw1();

  id[7] = (ID_1_3_val) >> 24;
  id[6] = (ID_1_3_val) >> 16;
  id[5] = (ID_1_3_val) >> 8;
  id[4] = (ID_1_3_val);
  id[3] = (ID_2_val) >> 24;
  id[2] = (ID_2_val) >> 16;
  id[1] = (ID_2_val) >> 8;
  id[0] = (ID_2_val);

  /* USER CODE BEGIN GetUniqueId_2 */

  /* USER CODE END GetUniqueId_2 */
}

uint32_t GetDevAddr(void)
{
  return ((HAL_GetUIDw0()) ^ (HAL_GetUIDw1()) ^ (HAL_GetUIDw2()));
}

/* USER CODE BEGIN ExF */
void GetVibrationData(vibrationStat_t* vibrationData)
{
	SYS_GetVibrationData(vibrationData);
}
/* USER CODE END ExF */

/* Private functions ---------------------------------------------------------*/


void Gpio_PreInit(void)
{


	/* USER CODE BEGIN Gpio_PreInit_1 */

	  /* USER CODE END Gpio_PreInit_1 */
	  GPIO_InitTypeDef GPIO_InitStruct = {0};

	  /* Configure all IOs in analog input              */
	  /* Except PA143 and PA14 (SWCLK and SWD) for debug*/
	  /* PA13 and PA14 are configured in debug_init     */
	  /* Configure all GPIO as analog to reduce current consumption on non used IOs */
	  /* Enable GPIOs clock */
	  __HAL_RCC_GPIOA_CLK_ENABLE();
	  __HAL_RCC_GPIOB_CLK_ENABLE();
	  __HAL_RCC_GPIOC_CLK_ENABLE();
	  __HAL_RCC_GPIOH_CLK_ENABLE();

	  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  /* All GPIOs except debug pins (SWCLK and SWD) */
	  GPIO_InitStruct.Pin = GPIO_PIN_All & (~(GPIO_PIN_13 | GPIO_PIN_14 ));//| VBUS_DETECT_Pin | LED_CNTRL_Pin));
	  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	  /* All GPIOs */
	  GPIO_InitStruct.Pin = GPIO_PIN_All ; //& (~(SENSOR_PWR_CNTRL_Pin));
	  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

	  /* Disable GPIOs clock */
	  __HAL_RCC_GPIOA_CLK_DISABLE();
	  __HAL_RCC_GPIOB_CLK_DISABLE();
	  __HAL_RCC_GPIOC_CLK_DISABLE();
	  __HAL_RCC_GPIOH_CLK_DISABLE();
	  /* USER CODE BEGIN Gpio_PreInit_2 */

	  /* USER CODE END Gpio_PreInit_2 */

}

/* Disable StopMode when traces need to be printed */
void UTIL_ADV_TRACE_PreSendHook(void)
{
  /* USER CODE BEGIN UTIL_ADV_TRACE_PreSendHook_1 */

  /* USER CODE END UTIL_ADV_TRACE_PreSendHook_1 */
  UTIL_LPM_SetStopMode((1 << CFG_LPM_UART_TX_Id), UTIL_LPM_DISABLE);
  /* USER CODE BEGIN UTIL_ADV_TRACE_PreSendHook_2 */

  /* USER CODE END UTIL_ADV_TRACE_PreSendHook_2 */
}
/* Re-enable StopMode when traces have been printed */
void UTIL_ADV_TRACE_PostSendHook(void)
{
  /* USER CODE BEGIN UTIL_LPM_SetStopMode_1 */

  /* USER CODE END UTIL_LPM_SetStopMode_1 */
  UTIL_LPM_SetStopMode((1 << CFG_LPM_UART_TX_Id), UTIL_LPM_ENABLE);
  /* USER CODE BEGIN UTIL_LPM_SetStopMode_2 */

  /* USER CODE END UTIL_LPM_SetStopMode_2 */
}


/* USER CODE BEGIN PrFD */

/* USER CODE END PrFD */
/* HAL overload functions ---------------------------------------------------------*/

/**
  * @brief This function configures the source of the time base.
  * @brief  don't enable systick
  * @param TickPriority: Tick interrupt priority.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority)
{
  /*Don't enable SysTick if TIMER_IF is based on other counters (e.g. RTC) */
  /* USER CODE BEGIN HAL_InitTick_1 */

  /* USER CODE END HAL_InitTick_1 */
  return HAL_OK;
  /* USER CODE BEGIN HAL_InitTick_2 */

  /* USER CODE END HAL_InitTick_2 */
}

/**
  * @brief Provide a tick value in millisecond measured using RTC
  * @note This function overwrites the __weak one from HAL
  * @retval tick value
  */
uint32_t HAL_GetTick(void)
{
  /* USER CODE BEGIN HAL_GetTick_1 */

  /* USER CODE END HAL_GetTick_1 */
  return RTC_IF_GetTimerValue();
  /* USER CODE BEGIN HAL_GetTick_2 */

  /* USER CODE END HAL_GetTick_2 */
}

/**
  * @brief This function provides delay (in ms)
  * @param Delay: specifies the delay time length, in milliseconds.
  * @retval None
  */
void HAL_Delay(__IO uint32_t Delay)
{
  /* USER CODE BEGIN HAL_Delay_1 */

  /* USER CODE END HAL_Delay_1 */
  RTC_IF_DelayMs(Delay);   /* based on RTC */
  /* USER CODE BEGIN HAL_Delay_2 */

  /* USER CODE END HAL_Delay_2 */
}

/* USER CODE BEGIN Overload_HAL_weaks */

/* USER CODE END Overload_HAL_weaks */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/


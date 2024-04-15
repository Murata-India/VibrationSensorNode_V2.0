/**
  ******************************************************************************
  * @file    master_app.c
  * @author  MCD Application Team
  * @brief   Application of the AT Master
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
#include "platform.h"
#include "sys_app.h"
#include "vibration_data.h"
#include "master_app.h"
#include "lora_driver.h"
#include "rtc_if.h"
#include "stdio.h"


#include ATCMD_MODEM        /* preprocessing definition in sys_conf.h*/



/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* External variables ---------------------------------------------------------*/
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Private typedef -----------------------------------------------------------*/


/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* CAYENNE_LLP is myDevices Application server*/
#define CAYENNE_LPP
#define LPP_DATATYPE_DIGITAL_INPUT 0x0
#define LPP_DATATYPE_DIGITAL_OUTPUT 0x1
#define LPP_DATATYPE_HUMIDITY 0x68
#define LPP_DATATYPE_TEMPERATURE 0x67
#define LPP_DATATYPE_BAROMETER 0x73

/* Private variables ---------------------------------------------------------*/

static vibrationStat_t VibrationData;                            /* struct for vibration statistical data*/


#if USE_LRWAN_NS1
static bool user_button_flag = true;
#endif


/* Private function prototypes -----------------------------------------------*/

static void SensorMeasureData(sSendDataBinary_t *SendDataBinary);

/* load call backs*/
static LoRaDriverCallback_t LoRaDriverCallbacks = { SensorMeasureData,
                                                    NULL
                                                  };

#if USE_LRWAN_NS1
/**
  * @brief  EXTI line detection callbacks.
  * @param  GPIO_Pin: Specifies the pins connected to the EXTI line.
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /*HW_RTC_DelayMs(10);*/
  RTC_IF_DelayMs(10);
  if (!HAL_GPIO_ReadPin(USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN))
  {
    if (user_button_flag)
    {
      Modem_IO_DeInit();
      BSP_LED_DeInit(LED2);
      user_button_flag = false;
      while (1);
    }
    else
    {
      Modem_IO_Init();
      NVIC_SystemReset();
    }
  }
}
#endif

/* Private macro ------------- -----------------------------------------------*/

#ifdef CAYENNE_LPP
#define LORAWAN_APP_PORT           99;            /*LoRaWAN application port*/
#else
#define LORAWAN_APP_PORT           2;            /*LoRaWAN application port*/
#endif

#define LORAWAN_CONFIRMED_MSG      ENABLE         /*LoRaWAN confirmed messages*/

#define SENSORS_MEASURE_CYCLE      15000          /*Period to do sensors measurement*/

#define JOIN_MODE                  OTAA_JOIN_MODE   /*ABP_JOIN_MODE */ /*LoRaWan join methode*/

#ifdef USE_LRWAN_NS1
#define FREQ_BAND                  /*EU868*/ CN470PREQUEL
#endif


/* Init LoRa Driver modem parameters*/
#ifdef USE_LRWAN_NS1
static LoRaDriverParam_t LoRaDriverParam = {  SENSORS_MEASURE_CYCLE,  JOIN_MODE, FREQ_BAND};
#else
static LoRaDriverParam_t LoRaDriverParam = {  SENSORS_MEASURE_CYCLE,  JOIN_MODE};
#endif



/*!
  * Master context initialization following the LoRa modem used
  *
  */
void MasterApp_Init(void)
{

  Modem_IO_Init();

  /* if using sequencer uncomment the task creation */
  /*UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_Lora_fsm), UTIL_SEQ_RFU, Lora_fsm); */

  Lora_Ctx_Init(&LoRaDriverCallbacks, &LoRaDriverParam);
}

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/********************* LoRa Part Apllication **********************************/

/******************************************************************************
  * @brief SensorMeasureData
  * @param none
  * @return none
  ******************************************************************************/
union floatByteStuffing
  {
        float value;
        uint8_t floatBytes[4];
  };

union int16ByteStuffing
{
		int16_t value;
		uint8_t intBytes[2];
};

static void SensorMeasureData(sSendDataBinary_t *SendDataBinary)
{
	uint8_t index = 0;
	union floatByteStuffing floatingPoint;
	union int16ByteStuffing int16Bytes;

	HAL_GPIO_WritePin(SENSOR_PWR_CNTRL_GPIO_Port, SENSOR_PWR_CNTRL_Pin, GPIO_PIN_RESET); // Switch on  power to the sensor
	GetVibrationData(&VibrationData);
	int16_t temperatureLevel = (GetTemperatureLevel() * 100);
	//DBG_PRINTF("Temperature = %d\r",temperatureLevel);
	DBG_PRINTF("0,0,0,0,0,%d\r",temperatureLevel);
	HAL_GPIO_WritePin(SENSOR_PWR_CNTRL_GPIO_Port, SENSOR_PWR_CNTRL_Pin, GPIO_PIN_SET); // Switch off  power to the sensor

	int16Bytes.value = VibrationData.vRMS;
	SendDataBinary->Buffer[index++] = int16Bytes.intBytes[0];
	SendDataBinary->Buffer[index++] = int16Bytes.intBytes[1];

	int16Bytes.value = VibrationData.stdValue;
	SendDataBinary->Buffer[index++] = int16Bytes.intBytes[0];
	SendDataBinary->Buffer[index++] = int16Bytes.intBytes[1];

	int16Bytes.value = VibrationData.maxPeak;
	SendDataBinary->Buffer[index++] = int16Bytes.intBytes[0];
	SendDataBinary->Buffer[index++] = int16Bytes.intBytes[1];

	int16Bytes.value = temperatureLevel;
	SendDataBinary->Buffer[index++] = int16Bytes.intBytes[0];
	SendDataBinary->Buffer[index++] = int16Bytes.intBytes[1];

	SendDataBinary->Buffer[index++] = VibrationData.noOfFFTPeaks;

	for(int i=0; i<VibrationData.noOfFFTPeaks; i++)
	{
		int16Bytes.value = VibrationData.maxFFTAmplitudeArray[i];
		SendDataBinary->Buffer[index++] = int16Bytes.intBytes[0];
		SendDataBinary->Buffer[index++] = int16Bytes.intBytes[1];

		int16Bytes.value = VibrationData.maxFFTFrequencyArray[i];
		SendDataBinary->Buffer[index++] = int16Bytes.intBytes[0];
		SendDataBinary->Buffer[index++] = int16Bytes.intBytes[1];
	}

	SendDataBinary->DataSize = index;
	SendDataBinary->Port = LORAWAN_APP_PORT;
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t *file, uint32_t line)
{

  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

}

#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

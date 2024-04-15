/**
  ******************************************************************************
  * @file    adc_if.c
  * @author  MCD Application Team
  * @brief   Read status related to the chip (battery level, VREF, chip temperature)
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
#include <sys_app.h>
#include "../../App/Inc/fft.h"
#include "../../App/Inc/time_domain_statistics.h"
/* USER CODE END Includes */

/* External variables ---------------------------------------------------------*/
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/

/* Internal voltage reference, parameter VREFINT_CAL*/
#define VREFINT_CAL       ((uint16_t*) ((uint32_t) 0x1FF80078))

/* Internal temperature sensor: constants data used for indicative values in  */
/* this example. Refer to device datasheet for min/typ/max values.            */

/* Internal temperature sensor, parameter TS_CAL1: TS ADC raw data acquired at
 *a temperature of 110 DegC (+-5 DegC), VDDA = 3.3 V (+-10 mV). */
#define TEMP30_CAL_ADDR   ((uint16_t*) ((uint32_t) 0x1FF8007A))

/* Internal temperature sensor, parameter TS_CAL2: TS ADC raw data acquired at
 *a temperature of  30 DegC (+-5 DegC), VDDA = 3.3 V (+-10 mV). */
#define TEMP110_CAL_ADDR  ((uint16_t*) ((uint32_t) 0x1FF8007E))

/* Vdda value with which temperature sensor has been calibrated in production
   (+-10 mV). */
#define VDDA_TEMP_CAL                  ((uint32_t) 3000)

/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
#define COMPUTE_TEMPERATURE(TS_ADC_DATA, VDDA_APPLI)                           \
  ((((( ((int32_t)((TS_ADC_DATA * VDDA_APPLI) / VDDA_TEMP_CAL)                  \
         - (int32_t) *TEMP30_CAL_ADDR)                                          \
      ) * (int32_t)(110 - 30)                                                   \
     )<<8) / (int32_t)(*TEMP110_CAL_ADDR - *TEMP30_CAL_ADDR)                        \
   ) + (30<<8)                                                                      \
  )

/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/**
  * @brief This function reads the ADC channel
  * @param channel channel number to read
  * @return adc measured level value
  */
static uint32_t ADC_ReadChannel_Temp(uint32_t channel);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Exported functions --------------------------------------------------------*/
/* USER CODE BEGIN EF */

/* USER CODE END EF */

void SYS_InitMeasurement(void)
{
  /* USER CODE BEGIN SYS_InitMeasurement_1 */

  /* USER CODE END SYS_InitMeasurement_1 */
  hadc1.Instance = ADC1;
  /* USER CODE BEGIN SYS_InitMeasurement_2 */

  /* USER CODE END SYS_InitMeasurement_2 */
}

void SYS_DeInitMeasurement(void)
{
  /* USER CODE BEGIN SYS_DeInitMeasurement_1 */

  /* USER CODE END SYS_DeInitMeasurement_1 */
}

float SYS_GetTemperatureLevel(void)
{
  /* USER CODE BEGIN SYS_GetTemperatureLevel_1 */

  /* USER CODE END SYS_GetTemperatureLevel_1 */
  float temperatureDegreeC = 0.0;
  /* Coefficients calculated using least square fitting in Matlab with the given data*/
  float THRM_A0 = 141.7502;
  float THRM_A1 = -178.9365;
  float THRM_A2 = 170.7529;
  float THRM_A3 = -99.0799;
  float THRM_A4 = 28.9903;
  float THRM_A5 = -3.3815;

  uint32_t measuredLevel = 0;
  float msdV = 0.0;
 // uint16_t batteryLevelmV = SYS_GetBatteryLevel();


  measuredLevel = ADC_ReadChannel_Temp(ADC_CHANNEL_11);
  msdV = (3.3/4096) * measuredLevel;
  temperatureDegreeC = (THRM_A5 * powf(msdV,5)) + (THRM_A4 * powf(msdV,4)) + (THRM_A3 * powf(msdV,3)) + (THRM_A2 * powf( msdV,2)) + (THRM_A1 *  msdV) + THRM_A0;
  return  temperatureDegreeC;
  /* USER CODE BEGIN SYS_GetTemperatureLevel_2 */

  /* USER CODE END SYS_GetTemperatureLevel_2 */
}

uint16_t SYS_GetBatteryLevel(void)
{
  /* USER CODE BEGIN SYS_GetBatteryLevel_1 */

  /* USER CODE END SYS_GetBatteryLevel_1 */
  uint16_t batteryLevelmV = 0;
  uint32_t measuredLevel = 0;


  measuredLevel = ADC_ReadChannels(ADC_CHANNEL_VREFINT);

  if (measuredLevel == 0)
  {
    batteryLevelmV = 0;
  }
  else
  {
    batteryLevelmV = (((uint32_t) VDDA_VREFINT_CAL * (*VREFINT_CAL)) / measuredLevel);
  }

  return batteryLevelmV;
  /* USER CODE BEGIN SYS_GetBatteryLevel_2 */

  /* USER CODE END SYS_GetBatteryLevel_2 */
}

/* Private Functions Definition -----------------------------------------------*/
/* USER CODE BEGIN PrFD */
void SYS_GetVibrationData(vibrationStat_t* vibrationData)
{

	MX_ADC1_Init();

    HAL_Delay(1000);

    memset(aADC1ConvertedValue_s, 0x00, sizeof(aADC1ConvertedValue_s[0])*SAMPLES);

	if(HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK) // Initialize the timer PWM for ADC sampling
	  	  Error_Handler();


	if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&aADC1ConvertedValue_s, SAMPLES) != HAL_OK) // Initialize the DMA
	{
	    /* Start Conversation Error */
		Error_Handler();
	}

	while(adcConversionFlag!=1)
	{
		HAL_Delay(50);
	}
	adcConversionFlag = 0;

	HAL_ADC_DeInit(&hadc1);

	//DBG_PRINTF("Raw Data\r");

	for(int i=0;i<SAMPLES;i++)
	{
		//DBG_PRINTF("%d\r",aADC1ConvertedValue_s[i]);
		aADC1EngineeringValue[i] = (((aADC1ConvertedValue_s[i]-1525)*3.43)/4096)/0.118;
		//DBG_PRINTF("%f\r",aADC1EngineeringValue[i]);
	}

    #if(DECIMATION_FACTOR != 1)
		FIR_DECIMATOR_F32Process();
    #endif

	/**Calculate Time domain Statistical Features**/
	TIME_DOMAIN_PROCESSING(vibrationData);
	/**Calculate Frequency domain Features**/
	FFT_PROCESSING_F32Process(vibrationData);

}
/* USER CODE END PrFD */

static uint32_t ADC_ReadChannel_Temp(uint32_t channel)
{
  /* USER CODE BEGIN ADC_ReadChannels_1 */

  /* USER CODE END ADC_ReadChannels_1 */
  uint32_t ADCxConvertedValues = 0;

  MX_ADC1_Init_Temp();

  if (HAL_ADC_Start(&hadc1) != HAL_OK)
  {
    /* Start Error */
    Error_Handler();
  }
  /** Wait for end of conversion */
  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);

  /** Wait for end of conversion */
  HAL_ADC_Stop(&hadc1) ;   /* it calls also ADC_Disable() */

  ADCxConvertedValues = HAL_ADC_GetValue(&hadc1);

  HAL_ADC_DeInit(&hadc1);

  __HAL_RCC_ADC_CLK_DISABLE();


  return ADCxConvertedValues;
  /* USER CODE BEGIN ADC_ReadChannels_2 */

  /* USER CODE END ADC_ReadChannels_2 */
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	HAL_ADC_Stop_DMA(&hadc1);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	adcConversionFlag = 1;
	//HAL_ADC_DeInit(&hadc1);
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

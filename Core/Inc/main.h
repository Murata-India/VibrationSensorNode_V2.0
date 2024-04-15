/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef hlpuart1;
extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;
extern TIM_HandleTypeDef htim1;

/*
 * Fixed hardware sampling rate
 * Caution : Do not change the HW sampling rate, as the analog anti-aliasing filter
 *           is designed for this sampling frequency.
 */
#define SAMPLING_RATE_HW     10000

/* Sample Length before Decimation  : Choose appropriate value to capture data for enough rotations.
 * Allowed Values: 512,1024,2048,4096
 */
#define SAMPLES    4096

/* Decimation filter:Allowed Values
 * 1 - Max Frequency 2.5 kHz
 * 2 - Max Frequency 1.25 kHz
 * 4 - Max Frequency 625 Hz
 */
#define DECIMATION_FACTOR      1

/*
 * Sample Length and sampling frequency after Decimation
 */
#define NEW_SAMPLE_LENGTH      ((SAMPLES)/(DECIMATION_FACTOR))
#define SAMPLING_RATE_SW       ((SAMPLING_RATE_HW)/(DECIMATION_FACTOR))

#define TIM1_CLOCK_FREQ    HAL_RCC_GetPCLK2Freq()
#define COUNTER_PERIOD     ((1/((float)(SAMPLING_RATE_SW)))/(1/((float)(TIM1_CLOCK_FREQ))))

#define FFT_BIN_WIDTH	   (((float)(SAMPLING_RATE_SW))/((float)(NEW_SAMPLE_LENGTH)))

//Threshold for peak detection
#define PEAK_MIN_HEIGHT  0
/* Minimum width between the peaks, peak that is present below the minimum width will be discarded*/
#define PEAK_MIN_WIDTH_HZ 100
#define PEAK_NO_OF_PEAKS  5
#define PEAK_MIN_WIDTH_BIN  (((float)(PEAK_MIN_WIDTH_HZ))/((float)(FFT_BIN_WIDTH)))
/*
 * Maximum Peak frequency of interest. Peaks above this value are discarded
 * Default value = (Sampling Rate/4) which is below the Nyquist limit of (Sampling Rate/2)
 * Max value is brought down below Nyquist limit to allow gradual filter roll-off (To reduce the number of filter taps)
 */
#define PEAK_MAX_FREQUENCY_HZ  ((int)((SAMPLING_RATE_SW)/(4)))

#define PEAK_MAX_FREQUENCY_BIN   ((int)((PEAK_MAX_FREQUENCY_HZ)/(FFT_BIN_WIDTH)))


extern uint16_t aADC1ConvertedValue_s [SAMPLES];
extern float aADC1EngineeringValue[SAMPLES];
extern volatile uint8_t adcConversionFlag;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
typedef enum
{
  LED1 = 0
} Led_TypeDef;
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
extern void MX_RTC_Init(void);
extern void SystemClock_Config(void);
extern void MX_GPIO_Init(void);
extern void MX_DMA_Init(void);
extern void MX_ADC1_Init(void);
extern void MX_TIM1_Init(void);
extern void MX_ADC1_Init_Temp(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define VBUS_DETECT_Pin GPIO_PIN_1
#define VBUS_DETECT_GPIO_Port GPIOA
#define SENSOR_PWR_CNTRL_Pin GPIO_PIN_10
#define SENSOR_PWR_CNTRL_GPIO_Port GPIOB
#define LED_CNTRL_Pin GPIO_PIN_8
#define LED_CNTRL_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */
#define LEDn                               1
#define USE_MDM32L07X01 1
#define RTC_N_PREDIV_S 10
#define RTC_PREDIV_S ((1<<RTC_N_PREDIV_S)-1)
#define RTC_PREDIV_A ((1<<(15-RTC_N_PREDIV_S))-1)
#define USART_BAUDRATE 115200
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

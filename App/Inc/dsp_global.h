/**
  ******************************************************************************
  * @file    STM32F429_DSPDEMO/Inc/global.h
  * @author  MCD Application Team
  * @brief   Header for global.c module
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GLOBAL_H
#define __GLOBAL_H
//#define __FPU_PRESENT             1U       /*!< FPU absent */

/* Includes ------------------------------------------------------------------*/
//#include "stm32l0xx_hal.h"
#include "main.h"

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "arm_math.h"
//#include <stdlib.h>
#include <string.h>

#include <math.h>
//#include "arm_math.h"

/* defines number of us per second */
#define US_IN_SECOND  					((uint32_t)1000000)

/* Systick Start */
#define TimerCount_Start()	do{\
							SysTick->LOAD  =  0xFFFFFF  ;	/* set reload register */\
							SysTick->VAL  =  0  ;			/* Clear Counter */		 \
							SysTick->CTRL  =  0x5 ;			/* Enable Counting*/	 \
							}while(0)

/* Systick Stop and retrieve CPU Clocks count */
#define TimerCount_Stop(Value) 	do {\
								SysTick->CTRL  =0;	/* Disable Counting */				 \
								Value = SysTick->VAL;/* Load the SysTick Counter Value */\
								Value = 0xFFFFFF - Value;/* Capture Counts in CPU Cycles*/\
								}while(0)
#define FIR_PROCESS 0
#define FFT_PROCESS 1

#define Float32 0
#define Q15 		1
#define Q31 		2
#define LPF			0
#define HPF			1

#define FFT_INVERSE_FLAG        ((uint8_t)0)
#define FFT_Normal_OUTPUT_FLAG  ((uint8_t)1)

/* Private variables ---------------------------------------------------------*/
//extern uint16_t aADC1ConvertedValue_s [512];
//extern uint32_t nb_cycles;
/* Private function prototypes -----------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

#endif /* __GLOBAL_H */
/************************ (C) COPYRIGHT STMicroelectronics ************************/

/**
  ******************************************************************************
  * @file    STM32F429_DSPDEMO/Inc/fft.h
  * @author  MCD Application Team
  * @brief   Header for fft_processing.c module
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
#ifndef __FFT_H
#define __FFT_H
/* Includes ------------------------------------------------------------------*/
#include "dsp_global.h"
#include "vibration_data.h"
#include "sys_debug.h"
/* Private defines -----------------------------------------------------------*/
#define FFT_LENGTH NEW_SAMPLE_LENGTH

#define minOfTwo(x,y) ((x) < (y) ? (x) : (y))
/* Private variables ---------------------------------------------------------*/
//extern int FFT_SIZE_CHOOSE;
/* Private function prototypes -----------------------------------------------*/
void FFT_PROCESSING_F32Process(vibrationStat_t* vibrationData);

void find_peaks(int32_t *pn_locs, int32_t *pn_npks, float32_t *pn_x, int32_t n_size, int32_t n_min_height, int32_t n_min_distance, int32_t n_max_num);
void peaks_above_min_height(int32_t *pn_locs, int32_t *pn_npks, float32_t  *pn_x, int32_t n_size, int32_t n_min_height);
void remove_close_peaks(int32_t *pn_locs, int32_t *pn_npks, float32_t *pn_x,int32_t n_min_distance);
void sort_ascend(int32_t *pn_x,int32_t n_size);
void sort_indices_descend(float32_t *pn_x, int32_t *pn_indx, int32_t n_size);

#endif /* __FFT_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

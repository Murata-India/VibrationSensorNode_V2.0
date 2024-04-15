/**
  ******************************************************************************
  * @file    STM32F429_DSPDEMO/Src/fir_processing.c
  * @author  MCD Application Team
  * @brief   Applying FIR Service Routines
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
/* Includes ------------------------------------------------------------------*/

#include "decimation_filter.h"

#if defined (DECIMATION_FACTOR) && (DECIMATION_FACTOR != 1)

#if defined (DECIMATION_FACTOR) && (DECIMATION_FACTOR == 2)
	#define  NUM_TAPS  25
	/* ----------------------------------------------------------------------
	FIR filter designed with
	http://t-filter.appspot.com

	sampling frequency: 10000 Hz

	* 0 Hz - 1250 Hz
	  gain = 1
	  desired ripple = 0.5 dB
	  actual ripple = 0.23429237384109897 dB

	* 2500 Hz - 5000 Hz
	  gain = 0
	  desired attenuation = -74 dB
	  actual attenuation = -78.01160609727867 dB
	** ------------------------------------------------------------------- */

	const float32_t filter_taps[NUM_TAPS] = {
	-0.001265551552727526, -0.004412649029130382,  -0.006264247487853215, -0.00025434645506315564, +0.014343622181724261,
	+0.023212825778927283, +0.005895626211228317,  -0.03558412004568527,  -0.058836794959691796,   -0.007980389485016323,
	+0.12672468722024066,  +0.27785653555893464,   +0.34435530498435846,  +0.27785653555893464,    +0.12672468722024066,
	-0.007980389485016323, -0.058836794959691796,  -0.03558412004568527,  +0.005895626211228317,   +0.023212825778927283,
	+0.014343622181724261, -0.00025434645506315564,-0.006264247487853215, -0.004412649029130382,   -0.001265551552727526
	};

#elif defined (DECIMATION_FACTOR) && (DECIMATION_FACTOR == 4)
	#define NUM_TAPS   35
	/* ----------------------------------------------------------------------
	FIR filter designed with
	http://t-filter.appspot.com

	sampling frequency: 10000 Hz

	* 0 Hz - 625 Hz
	  gain = 1
	  desired ripple = 0.5 dB
	  actual ripple = 0.27273787933670796 dB

	* 1500 Hz - 5000 Hz
	  gain = 0
	  desired attenuation = -74 dB
	  actual attenuation = -76.69204696303802 dB
	** ------------------------------------------------------------------- */

	const float32_t filter_taps[NUM_TAPS] = {
	+0.0004941881757643454, +0.0016038742416190498, +0.003483114244820776, +0.005697335511070282, +0.007152996555181008,
	+0.006282296290283656,  +0.0016912713121734163, -0.006881059254129872, -0.017691512258200964, -0.026733085376583837,
	-0.028474517119179164,  -0.017597137981971888,  +0.008777433529624923, +0.049245330970138564, +0.09755599569835652,
	+0.14381027203530136,   +0.1771984926793567,    +0.1893729211046614,   +0.1771984926793567,   +0.14381027203530136,
	+0.09755599569835652,   +0.049245330970138564,  +0.008777433529624923, -0.017597137981971888, -0.028474517119179164,
	-0.026733085376583837,  -0.017691512258200964,  -0.006881059254129872, +0.0016912713121734163,+0.006282296290283656,
	+0.007152996555181008,  +0.005697335511070282,  +0.003483114244820776, +0.0016038742416190498,+0.0004941881757643454
	};

#endif

uint32_t blockSize = BLOCK_SIZE;
uint32_t numBlocks = SAMPLES_LENGTH/BLOCK_SIZE;

float32_t 	aFIR_F32_Output[((SAMPLES_LENGTH)/(DECIMATION_FACTOR))];


/* ----------------------------------------------------------------------
** Test input signal contains 1000Hz + 15000 Hz
** ------------------------------------------------------------------- */
const float32_t  aFIR_F32_1kHz_15kHz[SAMPLES_LENGTH] =
{
+0.0000000000f, +0.5924659585f, -0.0947343455f, +0.1913417162f, +1.0000000000f, +0.4174197128f, +0.3535533906f, +1.2552931065f,
+0.8660254038f, +0.4619397663f, +1.3194792169f, +1.1827865776f, +0.5000000000f, +1.1827865776f, +1.3194792169f, +0.4619397663f,
+0.8660254038f, +1.2552931065f, +0.3535533906f, +0.4174197128f, +1.0000000000f, +0.1913417162f, -0.0947343455f, +0.5924659585f,
-0.0000000000f, -0.5924659585f, +0.0947343455f, -0.1913417162f, -1.0000000000f, -0.4174197128f, -0.3535533906f, -1.2552931065f,
-0.8660254038f, -0.4619397663f, -1.3194792169f, -1.1827865776f, -0.5000000000f, -1.1827865776f, -1.3194792169f, -0.4619397663f,
-0.8660254038f, -1.2552931065f, -0.3535533906f, -0.4174197128f, -1.0000000000f, -0.1913417162f, +0.0947343455f, -0.5924659585f,
+0.0000000000f, +0.5924659585f, -0.0947343455f, +0.1913417162f, +1.0000000000f, +0.4174197128f, +0.3535533906f, +1.2552931065f,
+0.8660254038f, +0.4619397663f, +1.3194792169f, +1.1827865776f, +0.5000000000f, +1.1827865776f, +1.3194792169f, +0.4619397663f,
+0.8660254038f, +1.2552931065f, +0.3535533906f, +0.4174197128f, +1.0000000000f, +0.1913417162f, -0.0947343455f, +0.5924659585f,
+0.0000000000f, -0.5924659585f, +0.0947343455f, -0.1913417162f, -1.0000000000f, -0.4174197128f, -0.3535533906f, -1.2552931065f,
-0.8660254038f, -0.4619397663f, -1.3194792169f, -1.1827865776f, -0.5000000000f, -1.1827865776f, -1.3194792169f, -0.4619397663f,
-0.8660254038f, -1.2552931065f, -0.3535533906f, -0.4174197128f, -1.0000000000f, -0.1913417162f, +0.0947343455f, -0.5924659585f,
+0.0000000000f, +0.5924659585f, -0.0947343455f, +0.1913417162f, +1.0000000000f, +0.4174197128f, +0.3535533906f, +1.2552931065f,
+0.8660254038f, +0.4619397663f, +1.3194792169f, +1.1827865776f, +0.5000000000f, +1.1827865776f, +1.3194792169f, +0.4619397663f,
+0.8660254038f, +1.2552931065f, +0.3535533906f, +0.4174197128f, +1.0000000000f, +0.1913417162f, -0.0947343455f, +0.5924659585f,
+0.0000000000f, -0.5924659585f, +0.0947343455f, -0.1913417162f, -1.0000000000f, -0.4174197128f, -0.3535533906f, -1.2552931065f,
-0.8660254038f, -0.4619397663f, -1.3194792169f, -1.1827865776f, -0.5000000000f, -1.1827865776f, -1.3194792169f, -0.4619397663f,
-0.8660254038f, -1.2552931065f, -0.3535533906f, -0.4174197128f, -1.0000000000f, -0.1913417162f, +0.0947343455f, -0.5924659585f,
-0.0000000000f, +0.5924659585f, -0.0947343455f, +0.1913417162f, +1.0000000000f, +0.4174197128f, +0.3535533906f, +1.2552931065f,
+0.8660254038f, +0.4619397663f, +1.3194792169f, +1.1827865776f, +0.5000000000f, +1.1827865776f, +1.3194792169f, +0.4619397663f,
+0.8660254038f, +1.2552931065f, +0.3535533906f, +0.4174197128f, +1.0000000000f, +0.1913417162f, -0.0947343455f, +0.5924659585f,
-0.0000000000f, -0.5924659585f, +0.0947343455f, -0.1913417162f, -1.0000000000f, -0.4174197128f, -0.3535533906f, -1.2552931065f,
-0.8660254038f, -0.4619397663f, -1.3194792169f, -1.1827865776f, -0.5000000000f, -1.1827865776f, -1.3194792169f, -0.4619397663f,
-0.8660254038f, -1.2552931065f, -0.3535533906f, -0.4174197128f, -1.0000000000f, -0.1913417162f, +0.0947343455f, -0.5924659585f,
+0.0000000000f, +0.5924659585f, -0.0947343455f, +0.1913417162f, +1.0000000000f, +0.4174197128f, +0.3535533906f, +1.2552931065f,
+0.8660254038f, +0.4619397663f, +1.3194792169f, +1.1827865776f, +0.5000000000f, +1.1827865776f, +1.3194792169f, +0.4619397663f,
+0.8660254038f, +1.2552931065f, +0.3535533906f, +0.4174197128f, +1.0000000000f, +0.1913417162f, -0.0947343455f, +0.5924659585f,
+0.0000000000f, -0.5924659585f, +0.0947343455f, -0.1913417162f, -1.0000000000f, -0.4174197128f, -0.3535533906f, -1.2552931065f,
-0.8660254038f, -0.4619397663f, -1.3194792169f, -1.1827865776f, -0.5000000000f, -1.1827865776f, -1.3194792169f, -0.4619397663f,
-0.8660254038f, -1.2552931065f, -0.3535533906f, -0.4174197128f, -1.0000000000f, -0.1913417162f, +0.0947343455f, -0.5924659585f,
};


/* -------------------------------------------------------------------
 * Declare State buffer of size (numTaps + blockSize - 1)
 * ------------------------------------------------------------------- */
float32_t  firStateF32[BLOCK_SIZE + NUM_TAPS - 1];

/**
  * @brief  This function apply a LP FIR filter in to a F32 data signal.
  * @param  None
  * @retval None
  */

void FIR_DECIMATOR_F32Process()
{
	arm_fir_decimate_instance_f32 FIR_DECIMATE_F32_Struct;

	uint32_t counter_FIR_f32_p;

	arm_fir_decimate_init_f32 (&FIR_DECIMATE_F32_Struct, NUM_TAPS, (uint8_t)DECIMATION_FACTOR, (float32_t *)&filter_taps[0],&firStateF32[0], blockSize);

    for (counter_FIR_f32_p = 0; counter_FIR_f32_p < numBlocks; counter_FIR_f32_p++)
    {
    	arm_fir_decimate_f32(&FIR_DECIMATE_F32_Struct, aFIR_F32_1kHz_15kHz + (counter_FIR_f32_p * blockSize),aFIR_F32_Output + (counter_FIR_f32_p * ((blockSize)/(DECIMATION_FACTOR))),blockSize);
    }

}
#endif
/*
 * decimation_filter.c
 *
 *  Created on: Mar 10, 2023
 *      Author: MIEB2001
 */


/**
  ******************************************************************************
  * @file    STM32F429_DSPDEMO/Src/fft_processing.c
  * @author  MCD Application Team
  * @brief   FFT calculation Service Routines
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
#include "fft.h"
#include "decimation_filter.h"

float32_t aFFT_Input_f32[FFT_LENGTH*2];
float32_t aFFT_Output_f32 [FFT_LENGTH];

/**
  * @brief  This function Calculate FFT in F32.
  * @param  FFT Length : 1024, 256, 64
  * @retval None ```` `                                                                                                                                                                                                                                                                                                                                                                                                                JU
  */
void FFT_PROCESSING_F32Process(vibrationStat_t* vibrationData)
{
  //arm_rfft_instance_f32 FFT_F32_struct;
  arm_cfft_radix2_instance_f32  FFT_F32_struct;

  //float32_t maxValueTemp;    /* Max FFT value is stored here */
  //uint32_t maxIndexTemp;    /* Index in Output array where max value is */
  int32_t an_dx_peak_locs[FFT_LENGTH/4] ;  // Peak location index
  int32_t n_npks;                //Total Number of max peaks

  unsigned index_fill_input_buffer, index_fill_adc_buffer = 0;
  uint32_t duration_us = 0x00;

  uint32_t nb_cycles;
  /*for (index_fill_adc_buffer = 0; index_fill_adc_buffer < FFT_Length*2; index_fill_adc_buffer ++)
  {
    aADC1ConvertedValue_s[index_fill_adc_buffer] = uhADCxConvertedValue;
    TIM2_Config();
  }*/
  float32_t hann_multiplier_buffer[FFT_LENGTH];
  float32_t windowed_data_buffer[FFT_LENGTH];
  for (int i = 0; i < FFT_LENGTH; i++) {
	  hann_multiplier_buffer[i] = 0.5 * (1 - cos(2*PI*i/(FFT_LENGTH-1)));
     //dataOut[i] = multiplier * dataIn[i];
  }

  #if defined (DECIMATION_FACTOR) && (DECIMATION_FACTOR != 1)
  	  arm_mult_f32(aFIR_F32_Output,hann_multiplier_buffer,windowed_data_buffer,FFT_LENGTH);
  #else
  	  arm_mult_f32(aADC1EngineeringValue,hann_multiplier_buffer,windowed_data_buffer,FFT_LENGTH);
  #endif

  for (index_fill_input_buffer = 0; index_fill_input_buffer < FFT_LENGTH*2; index_fill_input_buffer += 2, index_fill_adc_buffer++)
  {
    aFFT_Input_f32[(uint16_t)index_fill_input_buffer] = (float32_t)windowed_data_buffer[index_fill_adc_buffer];// / (float32_t)4096.0;
    /* Imaginary part */
    aFFT_Input_f32[(uint16_t)(index_fill_input_buffer + 1)] = 0;
    //TIM2_Config();
  }
  /* Initialize the CFFT/CIFFT module, intFlag = 0, doBitReverse = 1 */
  //arm_cfft_init_f32(&FFT_F32_struct,FFT_LENGTH);
  arm_cfft_radix2_init_f32(&FFT_F32_struct, FFT_LENGTH, FFT_INVERSE_FLAG, FFT_Normal_OUTPUT_FLAG);

  TimerCount_Start();
  arm_cfft_radix2_f32(&FFT_F32_struct, aFFT_Input_f32);
  TimerCount_Stop(nb_cycles);

  //GUI_Clear();
  //LCD_OUTPUT_Cycles(5, 305, nb_cycles);
  duration_us = (uint32_t)(((uint64_t)US_IN_SECOND * (nb_cycles)) / SystemCoreClock);
  //LCD_OUTPUT_DURATION(120, 305, duration_us);

  /* Process the data through the Complex Magnitude Module for calculating the magnitude at each bin */
  arm_cmplx_mag_f32(aFFT_Input_f32, aFFT_Output_f32, FFT_LENGTH);

  aFFT_Output_f32[0]= 0.0; //Eliminate DC component.
  //DBG_PRINTF("Frequency Bins \r");
  for(int i=0;i<FFT_LENGTH;i++)
  {
	  DBG_PRINTF("%f,%f,1,1,1,1\r",aADC1EngineeringValue[i],aFFT_Output_f32[i]);
  }


  /* Calculates maxValue and returns corresponding value */
  /*for(int k=0; k<5; k++)
  {
	  arm_max_f32(aFFT_Output_f32, (FFT_Length/2), &maxValueTemp, &maxIndexTemp);
	  vibrationData->maxFFTAmplitudeArray[k]= maxValueTemp;
	  vibrationData->maxFFTIndexArray[k] = maxIndexTemp ;
	  printf("\nMAX Value %d  = %f, Max Index = %d\n",k,maxValueTemp,(int)maxIndexTemp);
	  aFFT_Output_f32[maxIndexTemp]= 0.0;
  }*/
  //DBG_PRINTF("TIM 1 clock frequency = %d\n",(int)(TIM1_CLOCK_FREQ));
  find_peaks(an_dx_peak_locs, &n_npks, aFFT_Output_f32,PEAK_MAX_FREQUENCY_BIN,PEAK_MIN_HEIGHT,PEAK_MIN_WIDTH_BIN,PEAK_NO_OF_PEAKS);//peak_height, peak_distance, max_num_peaks
  for(int i=0; i<n_npks; i++)
  {
	 vibrationData->maxFFTAmplitudeArray[i]= aFFT_Output_f32[(an_dx_peak_locs[i])] * 100 ;
	 vibrationData->maxFFTFrequencyArray[i] = (an_dx_peak_locs[i] * FFT_BIN_WIDTH);
	 DBG_PRINTF("1,1,1,%f,%d,%d\r",aFFT_Output_f32[an_dx_peak_locs[i]], an_dx_peak_locs[i],vibrationData->maxFFTFrequencyArray[i]);
	 //DBG_PRINTF("%f @ %d @ %d Hz\r",aFFT_Output_f32[an_dx_peak_locs[i]], an_dx_peak_locs[i],vibrationData->maxFFTFrequencyArray[i]);
  }
  vibrationData->noOfFFTPeaks = n_npks;

  /*arm_max_f32(aFFT_Output_f32, (FFT_Length/2), &maxValue, &maxIndex);
  printf("\nMAX Value 1  = %f, Max Index = %d\n",maxValue, maxIndex);
  aFFT_Output_f32[maxIndex]= 0.0;
  arm_max_f32(aFFT_Output_f32, (FFT_Length/2), &maxValue, &maxIndex);
  printf("\nMAX Value 2 = %f, Max Index = %d\n",maxValue, maxIndex);
  aFFT_Output_f32[maxIndex]= 0.0;
  maxValue = 0;*/
}

void find_peaks(int32_t *pn_locs, int32_t *pn_npks, float32_t *pn_x, int32_t n_size, int32_t n_min_height, int32_t n_min_distance, int32_t n_max_num)
/**
* \brief        Find peaks
* \par          Details
*               Find at most MAX_NUM peaks above MIN_HEIGHT separated by at least MIN_DISTANCE
*
* \retval       None
*/
{
    peaks_above_min_height( pn_locs, pn_npks, pn_x, n_size, n_min_height );
    remove_close_peaks( pn_locs, pn_npks, pn_x, n_min_distance );
    *pn_npks = minOfTwo( *pn_npks, n_max_num );
}

void peaks_above_min_height(int32_t *pn_locs, int32_t *pn_npks, float32_t  *pn_x, int32_t n_size, int32_t n_min_height)
/**
* \brief        Find peaks above n_min_height
* \par          Details
*               Find all peaks above MIN_HEIGHT
*
* \retval       None
*/
{
    int32_t i = 1, n_width;
    *pn_npks = 0;

    while (i < n_size-1){
        if (pn_x[i] > n_min_height && pn_x[i] > pn_x[i-1]){            // find left edge of potential peaks
            n_width = 1;
            while (i+n_width < n_size && pn_x[i] == pn_x[i+n_width])    // find flat peaks
                n_width++;
            if (pn_x[i] > pn_x[i+n_width] && (*pn_npks) < (n_size/2) ){                            // find right edge of peaks
                pn_locs[(*pn_npks)++] = i;
                // for flat peaks, peak location is left edge
                i += n_width+1;
            }
            else
                i += n_width;
        }
        else
            i++;
    }
}

void remove_close_peaks(int32_t *pn_locs, int32_t *pn_npks, float32_t *pn_x,int32_t n_min_distance)
/**
* \brief        Remove peaks
* \par          Details
*               Remove peaks separated by less than MIN_DISTANCE
*
* \retval       None
*/
{

    int32_t i, j, n_old_npks, n_dist;

    /* Order peaks from large to small */
    sort_indices_descend(pn_x, pn_locs, *pn_npks );

    for ( i = -1; i < *pn_npks; i++ ){
        n_old_npks = *pn_npks;
        *pn_npks = i+1;
        for ( j = i+1; j < n_old_npks; j++ ){
            n_dist =  pn_locs[j] - ( i == -1 ? -1 : pn_locs[i] ); // lag-zero peak of autocorr is at index -1
            if ( n_dist > n_min_distance || n_dist < -n_min_distance )
                pn_locs[(*pn_npks)++] = pn_locs[j];
            else if(i == -1)
                {pn_locs[(*pn_npks)++] = pn_locs[j];}
        }
    }

    // Resort indices longo ascending order
  //  sort_ascend(pn_locs, *pn_npks );
}

void sort_ascend(int32_t *pn_x,int32_t n_size)
/**
* \brief        Sort array
* \par          Details
*               Sort array in ascending order (insertion sort algorithm)
*
* \retval       None
*/
{
    int32_t i, j;
    float32_t n_temp;
    for (i = 1; i < n_size; i++) {
        n_temp = pn_x[i];
        for (j = i; j > 0 && n_temp < pn_x[j-1]; j--)
            pn_x[j] = pn_x[j-1];
        pn_x[j] = n_temp;
    }
}

void sort_indices_descend(float32_t *pn_x, int32_t *pn_indx, int32_t n_size)
/**
* \brief        Sort indices
* \par          Details
*               Sort indices according to descending order (insertion sort algorithm)
*
* \retval       None
*/
{
    int32_t i, j, n_temp;
    for (i = 1; i < n_size; i++) {
        n_temp = pn_indx[i];
        for (j = i; j > 0 && pn_x[n_temp] > pn_x[pn_indx[j-1]]; j--)
            pn_indx[j] = pn_indx[j-1];
        pn_indx[j] = n_temp;
    }
}

/************************ (C) COPYRIGHT STMicroelectronics ************************/

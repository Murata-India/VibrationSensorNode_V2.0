/*
 * time_domain_statistics.c
 *
 *  Created on: 23-May-2022
 *      Author: MIEB2001
 */

#include "time_domain_statistics.h"
#include "decimation_filter.h"


void TIME_DOMAIN_PROCESSING(vibrationStat_t* vibrationData)
{
	//Calculate RMS
	//float32_t vRMS, stdValue, maxPeak, crestFactor;
	uint32_t maxPeakIndex;
	float vRMS,stdValue,maxPeak;

	#if defined (DECIMATION_FACTOR) && (DECIMATION_FACTOR != 1)
		arm_rms_f32 (aFIR_F32_Output, NEW_SAMPLE_LENGTH, &vRMS);
		arm_std_f32 (aFIR_F32_Output, NEW_SAMPLE_LENGTH , &stdValue);
		arm_max_f32 (aFIR_F32_Output, NEW_SAMPLE_LENGTH, &maxPeak, &maxPeakIndex);
	#else
		arm_rms_f32 (aADC1EngineeringValue, NEW_SAMPLE_LENGTH, &vRMS);
		arm_std_f32 (aADC1EngineeringValue, NEW_SAMPLE_LENGTH , &stdValue);
		arm_max_f32 (aADC1EngineeringValue, NEW_SAMPLE_LENGTH, &maxPeak, &maxPeakIndex);
    #endif

	vibrationData->vRMS = (vRMS * 100);
	vibrationData->stdValue = stdValue * 100;
	vibrationData->maxPeak = maxPeak *100;
	//Calculate Kurtosis
	//Skewness
	//Calculate Crest Factor
	//vibrationData->crestFactor = vibrationData->maxPeak/vibrationData->vRMS;
	DBG_PRINTF("%f,%f,%f,%f,0,0\r",vRMS,stdValue,maxPeak,maxPeakIndex);
	//DBG_PRINTF("RMS = %f,Standard Deviation = %f, Max Peak = %f @ %lu\r",vRMS,stdValue,maxPeak,maxPeakIndex);
}

/*
 * decimation_filter.h
 *
 *  Created on: Mar 10, 2023
 *      Author: MIEB2001
 */

#ifndef INC_DECIMATION_FILTER_H_
#define INC_DECIMATION_FILTER_H_

/* Includes ------------------------------------------------------------------*/
#include "dsp_global.h"
/* Private defines -----------------------------------------------------------*/
#define SAMPLES_LENGTH                  SAMPLES  /* Allowed values are 512, 1024, 2048, 4096*/
#define BLOCK_SIZE                      32

#define BLOCKSIZE                       32

/* Exported functions ------------------------------------------------------- */
void FIR_DECIMATOR_F32Process(void);
/* Private variables ---------------------------------------------------------*/
extern float32_t aFIR_F32_Output[((SAMPLES_LENGTH)/(DECIMATION_FACTOR))];
#endif /* INC_DECIMATION_FILTER_H_ */

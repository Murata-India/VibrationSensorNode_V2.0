/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __VIBRATION_H__
#define __VIBRATION_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
#include "arm_math.h"

typedef struct
{
  int16_t vRMS;
  int16_t stdValue;
  int16_t maxPeak;
  uint8_t noOfFFTPeaks;
  int16_t maxFFTAmplitudeArray[15];
  int16_t maxFFTFrequencyArray[15];
} vibrationStat_t;

#ifdef __cplusplus
}
#endif

#endif /* __VIBRATION_H__ */

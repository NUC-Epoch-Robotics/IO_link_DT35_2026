#ifndef _IOL_UTILS_H_
#define _IOL_UTILS_H_

#include "main.h"

uint8_t IOL_CalculateChecksum(uint8_t *pData, uint8_t length);
uint32_t IOL_CycleTime_To_us(uint8_t cycle_time_byte);

#endif

#include "iol_utils.h"

#define IOL_CKS_SEED 0x52U

uint8_t IOL_CalculateChecksum(uint8_t *pData, uint8_t length)
{
    uint8_t checksum = IOL_CKS_SEED;

    for (uint8_t i = 0U; i < length; i++)
    {
        checksum ^= pData[i];
    }

    checksum = (uint8_t)(
        (((checksum >> 7) ^ (checksum >> 5) ^ (checksum >> 3) ^ (checksum >> 1)) & 0x01U) << 5 |
        (((checksum >> 6) ^ (checksum >> 4) ^ (checksum >> 2) ^ checksum) & 0x01U) << 4 |
        (((checksum >> 7) ^ (checksum >> 6)) & 0x01U) << 3 |
        (((checksum >> 5) ^ (checksum >> 4)) & 0x01U) << 2 |
        (((checksum >> 3) ^ (checksum >> 2)) & 0x01U) << 1 |
        (((checksum >> 1) ^ checksum) & 0x01U));

    return checksum;
}

uint32_t IOL_CycleTime_To_us(uint8_t cycle_time_byte)
{
    uint8_t encoding = (uint8_t)((cycle_time_byte >> 6) & 0x03U);
    uint8_t multiplier = (uint8_t)(cycle_time_byte & 0x3FU);

    switch (encoding)
    {
        case 0U:
            return (uint32_t)multiplier * 100U;
        case 1U:
            return 6400U + ((uint32_t)multiplier * 400U);
        case 2U:
            return 32000U + ((uint32_t)multiplier * 1600U);
        default:
            return 0U;
    }
}

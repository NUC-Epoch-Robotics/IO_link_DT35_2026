#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "dt35_protocol.h"

static uint16_t le16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

int main(void)
{
    uint8_t frame[DT35_FRAME_V2_LEN];
    const uint8_t status[4] = {0xB0U, 0xB0U, 0xB0U, 0xB0U};
    const uint8_t age[4] = {1U, 2U, 3U, 4U};
    const uint16_t mm[4] = {1000U, 2000U, 3000U, 50000U};
    const uint16_t seq[4] = {10U, 20U, 30U, 40U};
    int fail = 0;

    if (DT35_Crc16CcittFalse((const uint8_t *)"123456789", 9U) != 0x29B1U) fail++;
    DT35_BuildStatusFrame(frame, 0x1234U, 0x0FU, status, age, mm, seq);
    if (frame[0] != 0xFFU || frame[1] != 0x55U || frame[2] != 0x02U || frame[3] != 0x01U) fail++;
    if (frame[6] != 25U || frame[7] != 0x0FU) fail++;
    if (le16(&frame[4]) != 0x1234U || frame[12] != 1U || frame[15] != 4U) fail++;
    if (le16(&frame[16]) != 1000U || le16(&frame[22]) != 50000U) fail++;
    if (le16(&frame[24]) != 10U || le16(&frame[30]) != 40U) fail++;
    if (le16(&frame[32]) != DT35_Crc16CcittFalse(frame, 32U)) fail++;

    if (fail == 0) {
        printf("dt35 protocol: all checks passed\n");
        return 0;
    }
    printf("dt35 protocol: %d failure(s)\n", fail);
    return 1;
}

#include "dt35_protocol.h"
#include <string.h>

static void dt35_write_le16(uint8_t *dst, uint16_t value)
{
    dst[0] = (uint8_t)(value & 0xFFU);
    dst[1] = (uint8_t)(value >> 8);
}

uint16_t DT35_Crc16CcittFalse(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFU;
    uint16_t i;
    uint8_t bit;

    if (data == 0) {
        return crc;
    }

    for (i = 0U; i < len; i++) {
        crc ^= (uint16_t)((uint16_t)data[i] << 8);
        for (bit = 0U; bit < 8U; bit++) {
            crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U)
                                  : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

void DT35_BuildStatusFrame(uint8_t out[DT35_FRAME_V2_LEN],
                           uint16_t frame_seq,
                           uint8_t valid_mask,
                           const uint8_t port_status[4],
                           const uint8_t age_ms[4],
                           const uint16_t distance_mm[4],
                           const uint16_t sample_seq[4])
{
    uint8_t i;
    uint16_t crc;

    if ((out == 0) || (port_status == 0) || (age_ms == 0) ||
        (distance_mm == 0) || (sample_seq == 0)) {
        return;
    }

    memset(out, 0, DT35_FRAME_V2_LEN);
    out[0] = 0xFFU;
    out[1] = 0x55U;
    out[2] = DT35_FRAME_VERSION;
    out[3] = DT35_FRAME_TYPE_STATUS;
    dt35_write_le16(&out[4], frame_seq);
    out[6] = 25U;
    out[7] = valid_mask;

    for (i = 0U; i < 4U; i++) {
        out[8U + i] = port_status[i];
        out[12U + i] = age_ms[i];
        dt35_write_le16(&out[16U + (i * 2U)], distance_mm[i]);
        dt35_write_le16(&out[24U + (i * 2U)], sample_seq[i]);
    }

    crc = DT35_Crc16CcittFalse(out, 32U);
    dt35_write_le16(&out[32], crc);
}

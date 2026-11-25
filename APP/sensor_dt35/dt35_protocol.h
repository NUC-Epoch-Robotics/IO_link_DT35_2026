#ifndef DT35_PROTOCOL_H
#define DT35_PROTOCOL_H

#include <stdint.h>

#define DT35_FRAME_V2_LEN       34U
#define DT35_FRAME_VERSION      0x02U
#define DT35_FRAME_TYPE_STATUS  0x01U

uint16_t DT35_Crc16CcittFalse(const uint8_t *data, uint16_t len);
void DT35_BuildStatusFrame(uint8_t out[DT35_FRAME_V2_LEN],
                           uint16_t frame_seq,
                           uint8_t valid_mask,
                           const uint8_t port_status[4],
                           const uint8_t age_ms[4],
                           const uint16_t distance_mm[4],
                           const uint16_t sample_seq[4]);

#endif

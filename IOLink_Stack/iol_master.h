#ifndef __IOL_MASTER_H
#define __IOL_MASTER_H

#include <stdbool.h>
#include <stdint.h>

#define IOL_MASTER_OK        0
#define IOL_MASTER_ERR_CKS   1
#define IOL_MASTER_TIMEOUT   2
#define IOL_MASTER_ERR_HW    3
#define IOL_MASTER_BUSY      4
#define IOL_MASTER_ERR_PROTO 5
#define IOL_MASTER_PORT_COUNT 4U

typedef struct {
    uint32_t transactions;
    uint32_t timeouts;
    uint32_t checksum_errors;
    uint32_t protocol_errors;
    uint32_t hardware_errors;
    uint32_t busy_errors;
    uint16_t last_rx_len;
} IOL_MasterStats_t;

typedef struct {
    uint16_t ExpectedVendorID;
    uint32_t ExpectedDeviceID;
    uint8_t  (*InitSequence)(void);
    void     (*ParseProcessData)(uint8_t *pd_buf, uint8_t len);
} IOL_DeviceDriver_t;

typedef enum {
    IOL_DIR_WRITE = 0x00,
    IOL_DIR_READ  = 0x01
} IOL_RW_e;

typedef enum {
    IOL_CH_PROCESS    = 0x00,
    IOL_CH_PAGE       = 0x01,
    IOL_CH_DIAGNOSTIC = 0x02,
    IOL_CH_ISDU       = 0x03
} IOL_Channel_e;

typedef enum {
    IOL_TYPE_0 = 0x00,
    IOL_TYPE_1 = 0x01,
    IOL_TYPE_2 = 0x02
} IOL_MsgType_e;

static inline void IOL_Build_MasterCommand(uint8_t *pTxBuf,
                                           IOL_RW_e rw,
                                           IOL_Channel_e ch,
                                           uint8_t addr,
                                           IOL_MsgType_e type)
{
    pTxBuf[0] = (uint8_t)(((rw & 0x01U) << 7) | ((ch & 0x03U) << 5) | (addr & 0x1FU));
    pTxBuf[1] = (uint8_t)((type & 0x03U) << 6);
}

uint8_t IOL_Master_ReadDirectParameter(uint8_t port_index, uint8_t address, uint8_t *pDataOut);
uint8_t IOL_Master_WriteDirectParameter(uint8_t port_index, uint8_t address, uint8_t data);
uint8_t IOL_Master_ReadProcessData(uint8_t port_index, uint8_t *pProcessData, uint8_t pdLength);
uint8_t IOL_Master_ReadProcessDataEx(uint8_t port_index,
                                     IOL_MsgType_e type,
                                     uint8_t *pProcessData,
                                     uint8_t pdLength);

uint8_t IOL_Master_ISDURead(uint8_t port_index,
                            uint16_t index,
                            uint8_t subindex,
                            bool hasSubindex,
                            uint8_t *pDataOut,
                            uint8_t *pDataLenInOut);

uint8_t IOL_Master_ISDUWrite(uint8_t port_index,
                             uint16_t index,
                             uint8_t subindex,
                             bool hasSubindex,
                             const uint8_t *pData,
                             uint8_t dataLen);

uint8_t IOL_Read_Device_Identity(uint8_t port_index, uint16_t *pVendorID, uint32_t *pDeviceID);
void IOL_Master_GetStats(uint8_t port_index, IOL_MasterStats_t *out);

#endif

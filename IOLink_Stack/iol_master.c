#include "iol_master.h"
#include "iol_port_hal.h"
#include "iol_utils.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

#define IOL_EVT_TX   0x01U
#define IOL_EVT_RX   0x02U
#define IOL_EVT_ERR  0x04U
#define IOL_EVT_ALL  (IOL_EVT_TX | IOL_EVT_RX | IOL_EVT_ERR)

#define IOL_ISDU_REQ_TIMEOUT_MS  50U

typedef struct {
    uint8_t frame_len;
    uint8_t payload_offset;
    uint8_t payload_len;
} IOL_RxProfile_t;

typedef struct {
    volatile bool error;
    volatile bool busy;
    volatile uint16_t rx_len;
    TaskHandle_t waiter;
    uint8_t tx_buf[64];
    uint8_t rx_buf[64];
    IOL_MasterStats_t stats;
} IOL_MasterPortContext_t;

static IOL_MasterPortContext_t s_ports[IOL_PORT_COUNT] = {0};

static IOL_MasterPortContext_t *IOL_Master_Port(uint8_t port_index)
{
    return (port_index < IOL_PORT_COUNT) ? &s_ports[port_index] : NULL;
}

static void IOL_Master_NotifyFromISR(IOL_MasterPortContext_t *ctx, uint32_t bits)
{
    BaseType_t woken = pdFALSE;

    if ((ctx != NULL) && (ctx->waiter != NULL)) {
        (void)xTaskNotifyFromISR(ctx->waiter, bits, eSetBits, &woken);
        portYIELD_FROM_ISR(woken);
    }
}

static uint8_t IOL_CalculateIsduChkpdu(const uint8_t *pData, uint8_t length)
{
    uint8_t chkpdu = 0U;

    for (uint8_t i = 0U; i < length; i++) {
        chkpdu ^= pData[i];
    }
    return chkpdu;
}

static uint8_t IOL_CalculateRequestChecksum(const uint8_t *pFrame, uint8_t frameLenWithoutChecksum)
{
    return IOL_CalculateChecksum((uint8_t *)pFrame, frameLenWithoutChecksum);
}

static bool IOL_IsduChecksumValid(const uint8_t *pData, uint8_t length)
{
    return IOL_CalculateIsduChkpdu(pData, length) == 0U;
}

static uint8_t IOL_ValidateOuterChecksum(const IOL_MasterPortContext_t *ctx, uint16_t payloadLen)
{
    uint8_t expected_cks;
    uint8_t received_cks;

    if ((payloadLen == 0U) || (payloadLen >= ctx->rx_len)) {
        return IOL_MASTER_ERR_PROTO;
    }

    expected_cks = IOL_CalculateChecksum((uint8_t *)ctx->rx_buf, (uint8_t)payloadLen);
    received_cks = (uint8_t)(ctx->rx_buf[payloadLen] & 0x3FU);

    return (expected_cks == received_cks) ? IOL_MASTER_OK : IOL_MASTER_ERR_CKS;
}

static bool IOL_GetRxProfile(IOL_MsgType_e type, IOL_RxProfile_t *profile)
{
    if (profile == NULL) {
        return false;
    }

    switch (type) {
        case IOL_TYPE_0:
            profile->frame_len = 2U;
            profile->payload_offset = 1U;
            profile->payload_len = 0U;
            break;
        case IOL_TYPE_1:
            profile->frame_len = 3U;
            profile->payload_offset = 1U;
            profile->payload_len = 1U;
            break;
        case IOL_TYPE_2:
            profile->frame_len = 4U;
            profile->payload_offset = 1U;
            profile->payload_len = 2U;
            break;
        default:
            return false;
    }
    return true;
}

static void IOL_RecordStatus(uint8_t port_index, uint8_t status)
{
    IOL_MasterPortContext_t *ctx = IOL_Master_Port(port_index);

    if (ctx == NULL) {
        return;
    }

    switch (status) {
        case IOL_MASTER_TIMEOUT:
            ctx->stats.timeouts++;
            break;
        case IOL_MASTER_ERR_CKS:
            ctx->stats.checksum_errors++;
            break;
        case IOL_MASTER_ERR_PROTO:
            ctx->stats.protocol_errors++;
            break;
        case IOL_MASTER_ERR_HW:
            ctx->stats.hardware_errors++;
            break;
        case IOL_MASTER_BUSY:
            ctx->stats.busy_errors++;
            break;
        default:
            break;
    }
    ctx->stats.last_rx_len = ctx->rx_len;
}

static uint8_t IOL_SendOnRequestFrame(uint8_t port_index,
                                      IOL_RW_e rw,
                                      IOL_Channel_e channel,
                                      uint8_t address,
                                      IOL_MsgType_e type,
                                      const uint8_t *pOdData,
                                      uint8_t odLen,
                                      uint16_t minRxLen,
                                      uint16_t maxRxLen,
                                      uint32_t timeout_ms)
{
    IOL_MasterPortContext_t *ctx = IOL_Master_Port(port_index);

    (void)minRxLen;
    (void)maxRxLen;

    if ((ctx == NULL) || ((odLen > 0U) && (pOdData == NULL))) {
        return IOL_MASTER_ERR_HW;
    }

    IOL_Build_MasterCommand(ctx->tx_buf, rw, channel, address, type);
    if (odLen > 0U) {
        memcpy(&ctx->tx_buf[2], pOdData, odLen);
    }
    ctx->tx_buf[1] |= IOL_CalculateRequestChecksum(ctx->tx_buf, (uint8_t)(odLen + 2U));

    return IOL_MASTER_OK;
}

static uint8_t IOL_Execute_Transaction(uint8_t port_index,
                                       uint16_t txLen,
                                       uint16_t minRxLen,
                                       uint16_t maxRxLen,
                                       uint32_t timeout_ms)
{
    IOL_MasterPortContext_t *ctx = IOL_Master_Port(port_index);
    uint32_t start_tick;
    uint32_t ignored;
    bool echo_checked = false;
    uint8_t result = IOL_MASTER_TIMEOUT;

    if (ctx == NULL) {
        return IOL_MASTER_ERR_HW;
    }

    if (ctx->busy) {
        ctx->stats.busy_errors++;
        return IOL_MASTER_BUSY;
    }

    ctx->busy = true;
    ctx->error = false;
    ctx->rx_len = 0U;
    ctx->stats.transactions++;
    ctx->waiter = xTaskGetCurrentTaskHandle();
    (void)xTaskNotifyWait(0U, IOL_EVT_ALL, &ignored, 0U);

    if (IOL_HAL_Start_Transaction(port_index, ctx->tx_buf, txLen) != IOL_MASTER_OK) {
        result = IOL_MASTER_ERR_HW;
        IOL_HAL_AbortTransaction(port_index);
        goto done;
    }

    start_tick = HAL_GetTick();
    while ((HAL_GetTick() - start_tick) <= timeout_ms) {
        uint32_t events = 0U;
        uint16_t available;
        TickType_t elapsed = (TickType_t)(HAL_GetTick() - start_tick);
        TickType_t wait_ticks;
        TickType_t remaining;
        uint8_t echo[64];

        if (ctx->error) {
            result = IOL_MASTER_ERR_HW;
            IOL_HAL_AbortTransaction(port_index);
            goto done;
        }

        remaining = (TickType_t)(timeout_ms - elapsed);
        if (remaining == 0U) {
            break;
        }
        wait_ticks = pdMS_TO_TICKS(remaining);
        if (wait_ticks == 0U) {
            wait_ticks = 1U;
        }
        if (wait_ticks > 2U) {
            wait_ticks = 2U;
        }

        (void)xTaskNotifyWait(0U, IOL_EVT_ALL, &events, wait_ticks);
        available = IOL_HAL_GetRxAvailable(port_index);

        if (!echo_checked && (available >= txLen)) {
            IOL_HAL_PeekRx(port_index, echo, txLen);
            if (memcmp(echo, ctx->tx_buf, txLen) == 0) {
                IOL_HAL_AdvanceRx(port_index, txLen);
                available = IOL_HAL_GetRxAvailable(port_index);
            }
            echo_checked = true;
        }

        if (((events & IOL_EVT_RX) != 0U) && (available >= minRxLen)) {
            if (available > maxRxLen) {
                result = IOL_MASTER_ERR_PROTO;
                IOL_HAL_AbortTransaction(port_index);
                goto done;
            }

            if ((maxRxLen != minRxLen) || (available >= maxRxLen)) {
                ctx->rx_len = available;
                IOL_HAL_ReadRx(port_index, ctx->rx_buf, available);
                result = IOL_MASTER_OK;
                goto done;
            }
        }
    }

    IOL_HAL_AbortTransaction(port_index);

done:
    ctx->waiter = NULL;
    ctx->busy = false;
    IOL_RecordStatus(port_index, result);
    return result;
}

static uint8_t IOL_TransactFrame(uint8_t port_index,
                                 IOL_RW_e rw,
                                 IOL_Channel_e channel,
                                 uint8_t address,
                                 IOL_MsgType_e type,
                                 const uint8_t *pOdData,
                                 uint8_t odLen,
                                 uint16_t minRxLen,
                                 uint16_t maxRxLen,
                                 uint32_t timeout_ms)
{
    IOL_MasterPortContext_t *ctx = IOL_Master_Port(port_index);
    uint8_t status;

    if (ctx == NULL) {
        return IOL_MASTER_ERR_HW;
    }

    status = IOL_SendOnRequestFrame(port_index, rw, channel, address, type,
                                    pOdData, odLen, minRxLen, maxRxLen, timeout_ms);
    if (status != IOL_MASTER_OK) {
        IOL_RecordStatus(port_index, status);
        return status;
    }

    return IOL_Execute_Transaction(port_index,
                                   (uint16_t)(odLen + 2U),
                                   minRxLen,
                                   maxRxLen,
                                   timeout_ms);
}

void IOL_MAC_TxCpltCallback(uint8_t port_index)
{
    IOL_MasterPortContext_t *ctx = IOL_Master_Port(port_index);
    IOL_Master_NotifyFromISR(ctx, IOL_EVT_TX);
}

void IOL_MAC_RxIdleCallback(uint8_t port_index, uint16_t rxLength)
{
    IOL_MasterPortContext_t *ctx = IOL_Master_Port(port_index);

    if (ctx != NULL) {
        ctx->rx_len = rxLength;
        IOL_Master_NotifyFromISR(ctx, IOL_EVT_RX);
    }
}

void IOL_MAC_ErrorCallback(uint8_t port_index)
{
    IOL_MasterPortContext_t *ctx = IOL_Master_Port(port_index);

    if (ctx != NULL) {
        ctx->error = true;
        IOL_Master_NotifyFromISR(ctx, IOL_EVT_ERR);
    }
}

uint8_t IOL_Master_ReadDirectParameter(uint8_t port_index, uint8_t address, uint8_t *pDataOut)
{
    IOL_MasterPortContext_t *ctx = IOL_Master_Port(port_index);
    uint8_t status;

    if ((ctx == NULL) || (pDataOut == NULL)) {
        return IOL_MASTER_ERR_HW;
    }

    status = IOL_TransactFrame(port_index,
                               IOL_DIR_READ,
                               IOL_CH_PAGE,
                               address,
                               IOL_TYPE_0,
                               NULL,
                               0U,
                               2U,
                               2U,
                               10U);
    if (status != IOL_MASTER_OK) {
        return status;
    }

    if ((ctx->rx_len != 2U) || (IOL_ValidateOuterChecksum(ctx, 1U) != IOL_MASTER_OK)) {
        return IOL_MASTER_ERR_CKS;
    }

    *pDataOut = ctx->rx_buf[0];
    return IOL_MASTER_OK;
}

uint8_t IOL_Master_WriteDirectParameter(uint8_t port_index, uint8_t address, uint8_t data)
{
    IOL_MasterPortContext_t *ctx = IOL_Master_Port(port_index);
    uint8_t status;

    if (ctx == NULL) {
        return IOL_MASTER_ERR_HW;
    }

    status = IOL_TransactFrame(port_index,
                               IOL_DIR_WRITE,
                               IOL_CH_PAGE,
                               address,
                               IOL_TYPE_0,
                               &data,
                               1U,
                               1U,
                               3U,
                               10U);
    if (status != IOL_MASTER_OK) {
        return status;
    }

    if (ctx->rx_len == 1U) {
        return IOL_MASTER_OK;
    }
    if ((ctx->rx_len == 3U) && (IOL_ValidateOuterChecksum(ctx, 2U) == IOL_MASTER_OK)) {
        return IOL_MASTER_OK;
    }
    return IOL_MASTER_ERR_PROTO;
}

uint8_t IOL_Master_ReadProcessDataEx(uint8_t port_index,
                                     IOL_MsgType_e type,
                                     uint8_t *pProcessData,
                                     uint8_t pdLength)
{
    IOL_MasterPortContext_t *ctx = IOL_Master_Port(port_index);
    IOL_RxProfile_t profile;
    uint8_t status;

    if ((ctx == NULL) || (pProcessData == NULL) || !IOL_GetRxProfile(type, &profile)) {
        return IOL_MASTER_ERR_HW;
    }

    if (pdLength > profile.payload_len) {
        return IOL_MASTER_ERR_PROTO;
    }

    status = IOL_TransactFrame(port_index,
                               IOL_DIR_READ,
                               IOL_CH_PROCESS,
                               0x00U,
                               type,
                               NULL,
                               0U,
                               profile.frame_len,
                               profile.frame_len,
                               8U);
    if (status != IOL_MASTER_OK) {
        return status;
    }

    if (ctx->rx_len != profile.frame_len) {
        return IOL_MASTER_ERR_PROTO;
    }

    status = IOL_ValidateOuterChecksum(ctx, (uint16_t)(profile.frame_len - 1U));
    if (status != IOL_MASTER_OK) {
        return status;
    }

    if (pdLength > 0U) {
        memcpy(pProcessData, &ctx->rx_buf[profile.payload_offset], pdLength);
    }
    return IOL_MASTER_OK;
}

uint8_t IOL_Master_ReadProcessData(uint8_t port_index, uint8_t *pProcessData, uint8_t pdLength)
{
    return IOL_Master_ReadProcessDataEx(port_index, IOL_TYPE_2, pProcessData, pdLength);
}

uint8_t IOL_Master_ISDURead(uint8_t port_index,
                            uint16_t index,
                            uint8_t subindex,
                            bool hasSubindex,
                            uint8_t *pDataOut,
                            uint8_t *pDataLenInOut)
{
    IOL_MasterPortContext_t *ctx = IOL_Master_Port(port_index);
    uint8_t isdu_req[8];
    uint8_t isdu_req_len;
    uint8_t status;
    uint8_t isdu_rsp_len;
    uint8_t rsp_service;
    uint8_t rsp_len_field;
    uint8_t data_len;

    if ((ctx == NULL) || (pDataOut == NULL) || (pDataLenInOut == NULL)) {
        return IOL_MASTER_ERR_HW;
    }

    if (hasSubindex) {
        if (index > 0xFFU) {
            isdu_req[0] = 0xB5U;
            isdu_req[1] = (uint8_t)(index >> 8);
            isdu_req[2] = (uint8_t)(index & 0xFFU);
            isdu_req[3] = subindex;
            isdu_req[4] = 0U;
            isdu_req_len = 5U;
        } else {
            isdu_req[0] = 0xA4U;
            isdu_req[1] = (uint8_t)index;
            isdu_req[2] = subindex;
            isdu_req[3] = 0U;
            isdu_req_len = 4U;
        }
    } else {
        if (index > 0xFFU) {
            return IOL_MASTER_ERR_HW;
        }
        isdu_req[0] = 0x93U;
        isdu_req[1] = (uint8_t)index;
        isdu_req[2] = 0U;
        isdu_req_len = 3U;
    }

    isdu_req[isdu_req_len - 1U] = IOL_CalculateIsduChkpdu(isdu_req, (uint8_t)(isdu_req_len - 1U));

    status = IOL_TransactFrame(port_index,
                               IOL_DIR_READ,
                               IOL_CH_ISDU,
                               0x00U,
                               IOL_TYPE_2,
                               isdu_req,
                               isdu_req_len,
                               3U,
                               8U,
                               IOL_ISDU_REQ_TIMEOUT_MS);
    if (status != IOL_MASTER_OK) {
        return status;
    }

    if (ctx->rx_len < 3U) {
        return IOL_MASTER_ERR_PROTO;
    }

    status = IOL_ValidateOuterChecksum(ctx, (uint16_t)(ctx->rx_len - 1U));
    if (status != IOL_MASTER_OK) {
        return status;
    }

    isdu_rsp_len = (uint8_t)(ctx->rx_len - 1U);
    if (!IOL_IsduChecksumValid(ctx->rx_buf, isdu_rsp_len)) {
        return IOL_MASTER_ERR_CKS;
    }

    rsp_service = (uint8_t)(ctx->rx_buf[0] >> 4);
    rsp_len_field = (uint8_t)(ctx->rx_buf[0] & 0x0FU);

    if ((rsp_len_field != isdu_rsp_len) || (rsp_service != 0xDU) || (isdu_rsp_len < 2U)) {
        return IOL_MASTER_ERR_PROTO;
    }

    data_len = (uint8_t)(isdu_rsp_len - 2U);
    if (*pDataLenInOut < data_len) {
        return IOL_MASTER_ERR_HW;
    }

    memcpy(pDataOut, &ctx->rx_buf[1], data_len);
    *pDataLenInOut = data_len;
    return IOL_MASTER_OK;
}

uint8_t IOL_Master_ISDUWrite(uint8_t port_index,
                             uint16_t index,
                             uint8_t subindex,
                             bool hasSubindex,
                             const uint8_t *pData,
                             uint8_t dataLen)
{
    IOL_MasterPortContext_t *ctx = IOL_Master_Port(port_index);
    uint8_t isdu_req[32];
    uint8_t status;
    uint8_t req_len;
    uint8_t rsp_service;
    uint8_t rsp_len_field;

    if ((ctx == NULL) || ((dataLen > 0U) && (pData == NULL))) {
        return IOL_MASTER_ERR_HW;
    }

    if (hasSubindex) {
        if (index > 0xFFU) {
            req_len = (uint8_t)(5U + dataLen);
            if ((req_len > sizeof(isdu_req)) || (req_len > 15U)) {
                return IOL_MASTER_ERR_HW;
            }
            isdu_req[0] = (uint8_t)(0x30U | req_len);
            isdu_req[1] = (uint8_t)(index >> 8);
            isdu_req[2] = (uint8_t)(index & 0xFFU);
            isdu_req[3] = subindex;
            if (dataLen > 0U) {
                memcpy(&isdu_req[4], pData, dataLen);
            }
            isdu_req[req_len - 1U] = 0U;
            isdu_req[req_len - 1U] = IOL_CalculateIsduChkpdu(isdu_req, (uint8_t)(req_len - 1U));
        } else {
            req_len = (uint8_t)(4U + dataLen);
            if ((req_len > sizeof(isdu_req)) || (req_len > 15U)) {
                return IOL_MASTER_ERR_HW;
            }
            isdu_req[0] = (uint8_t)(0x20U | req_len);
            isdu_req[1] = (uint8_t)index;
            isdu_req[2] = subindex;
            if (dataLen > 0U) {
                memcpy(&isdu_req[3], pData, dataLen);
            }
            isdu_req[req_len - 1U] = 0U;
            isdu_req[req_len - 1U] = IOL_CalculateIsduChkpdu(isdu_req, (uint8_t)(req_len - 1U));
        }
    } else {
        if (index > 0xFFU) {
            return IOL_MASTER_ERR_HW;
        }
        req_len = (uint8_t)(3U + dataLen);
        if ((req_len > sizeof(isdu_req)) || (req_len > 15U)) {
            return IOL_MASTER_ERR_HW;
        }
        isdu_req[0] = (uint8_t)(0x10U | req_len);
        isdu_req[1] = (uint8_t)index;
        if (dataLen > 0U) {
            memcpy(&isdu_req[2], pData, dataLen);
        }
        isdu_req[req_len - 1U] = 0U;
        isdu_req[req_len - 1U] = IOL_CalculateIsduChkpdu(isdu_req, (uint8_t)(req_len - 1U));
    }

    status = IOL_TransactFrame(port_index,
                               IOL_DIR_WRITE,
                               IOL_CH_ISDU,
                               0x00U,
                               IOL_TYPE_2,
                               isdu_req,
                               req_len,
                               3U,
                               3U,
                               IOL_ISDU_REQ_TIMEOUT_MS);
    if (status != IOL_MASTER_OK) {
        return status;
    }

    if ((ctx->rx_len != 3U) || (IOL_ValidateOuterChecksum(ctx, 2U) != IOL_MASTER_OK) ||
        !IOL_IsduChecksumValid(ctx->rx_buf, 2U)) {
        return IOL_MASTER_ERR_PROTO;
    }

    rsp_service = (uint8_t)(ctx->rx_buf[0] >> 4);
    rsp_len_field = (uint8_t)(ctx->rx_buf[0] & 0x0FU);
    if ((rsp_service != 0x5U) || (rsp_len_field != 2U)) {
        return IOL_MASTER_ERR_PROTO;
    }

    return IOL_MASTER_OK;
}

uint8_t IOL_Read_Device_Identity(uint8_t port_index, uint16_t *pVendorID, uint32_t *pDeviceID)
{
    uint8_t byte_val = 0U;
    uint8_t status;

    if ((pVendorID == NULL) || (pDeviceID == NULL)) {
        return IOL_MASTER_ERR_HW;
    }

    *pVendorID = 0U;
    status = IOL_Master_ReadDirectParameter(port_index, 0x07U, &byte_val);
    if (status != IOL_MASTER_OK) return status;
    *pVendorID |= (uint16_t)((uint16_t)byte_val << 8);

    status = IOL_Master_ReadDirectParameter(port_index, 0x08U, &byte_val);
    if (status != IOL_MASTER_OK) return status;
    *pVendorID |= byte_val;

    *pDeviceID = 0U;
    status = IOL_Master_ReadDirectParameter(port_index, 0x09U, &byte_val);
    if (status != IOL_MASTER_OK) return status;
    *pDeviceID |= ((uint32_t)byte_val << 16);

    status = IOL_Master_ReadDirectParameter(port_index, 0x0AU, &byte_val);
    if (status != IOL_MASTER_OK) return status;
    *pDeviceID |= ((uint32_t)byte_val << 8);

    status = IOL_Master_ReadDirectParameter(port_index, 0x0BU, &byte_val);
    if (status != IOL_MASTER_OK) return status;
    *pDeviceID |= byte_val;

    return IOL_MASTER_OK;
}

void IOL_Master_GetStats(uint8_t port_index, IOL_MasterStats_t *out)
{
    IOL_MasterPortContext_t *ctx = IOL_Master_Port(port_index);

    if ((ctx != NULL) && (out != NULL)) {
        *out = ctx->stats;
    }
}

#include "iol_port_hal.h"
#include "iol_master.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

typedef struct {
    UART_HandleTypeDef *huart;
    void (*init_fn)(void);
    GPIO_TypeDef *txen_port;
    uint16_t txen_pin;
    uint8_t ltc_port;
    uint8_t dma_rx_buf[IOL_RX_RING_SIZE];
    volatile uint32_t rx_read_total;
    volatile uint32_t rx_wrap_count;
    volatile bool dma_rx_started;
    volatile uint32_t dma_overrun_count;
    volatile uint32_t reinit_count;
} IOL_HalPortContext_t;

static IOL_HalPortContext_t s_ports[IOL_PORT_COUNT] = {
    {&huart5, MX_UART5_Init, TXEN1_GPIO_Port, TXEN1_Pin, 1U, {0U}, 0U, 0U, false, 0U, 0U},
    {&huart4, MX_UART4_Init, TXEN2_GPIO_Port, TXEN2_Pin, 2U, {0U}, 0U, 0U, false, 0U, 0U},
    {&huart3, MX_USART3_UART_Init, TXEN3_GPIO_Port, TXEN3_Pin, 3U, {0U}, 0U, 0U, false, 0U, 0U},
    {&huart6, MX_USART6_UART_Init, TXEN4_GPIO_Port, TXEN4_Pin, 4U, {0U}, 0U, 0U, false, 0U, 0U}
};

static SemaphoreHandle_t s_phy_mutex;

static void IOL_HAL_PhyLock(void)
{
    if (s_phy_mutex != NULL) {
        (void)xSemaphoreTake(s_phy_mutex, portMAX_DELAY);
    }
}

static void IOL_HAL_PhyUnlock(void)
{
    if (s_phy_mutex != NULL) {
        (void)xSemaphoreGive(s_phy_mutex);
    }
}

static IOL_HalPortContext_t *IOL_HAL_Port(uint8_t port_index)
{
    return (port_index < IOL_PORT_COUNT) ? &s_ports[port_index] : NULL;
}

static uint8_t IOL_HAL_IndexFromUart(const UART_HandleTypeDef *huart)
{
    uint8_t i;

    for (i = 0U; i < IOL_PORT_COUNT; i++) {
        if (s_ports[i].huart->Instance == huart->Instance) {
            return i;
        }
    }
    return 0xFFU;
}

static uint16_t IOL_HAL_GetRxWriteIndex(IOL_HalPortContext_t *ctx)
{
    uint16_t index;

    if ((ctx == NULL) || (ctx->huart->hdmarx == NULL)) {
        return 0U;
    }

    index = (uint16_t)(IOL_RX_RING_SIZE - __HAL_DMA_GET_COUNTER(ctx->huart->hdmarx));
    return (index < IOL_RX_RING_SIZE) ? index : 0U;
}

static uint32_t IOL_HAL_GetRxWriteTotal(IOL_HalPortContext_t *ctx)
{
    return (ctx->rx_wrap_count * IOL_RX_RING_SIZE) + IOL_HAL_GetRxWriteIndex(ctx);
}

static void IOL_HAL_ApplyPhyBaud(IOL_HalPortContext_t *ctx, uint32_t baudrate)
{
    uint8_t mode2;
    uint8_t nsf;
    uint8_t slew_mask;
    uint8_t nsf_mask;
    uint8_t nsf_com1;
    uint8_t nsf_com2;
    uint8_t nsf_com3;

    mode2 = LTC2874_SPI_ReadReg(LTC2874_REG_MODE2);
    nsf = LTC2874_SPI_ReadReg(LTC2874_REG_NSF);

    switch (ctx->ltc_port) {
        case 1U:
            slew_mask = LTC2874_MODE2_SLEW1;
            nsf_mask = LTC2874_NSF1_MASK;
            nsf_com1 = LTC2874_NSF1_COM1;
            nsf_com2 = LTC2874_NSF1_COM2;
            nsf_com3 = LTC2874_NSF1_COM3;
            break;
        case 2U:
            slew_mask = LTC2874_MODE2_SLEW2;
            nsf_mask = LTC2874_NSF2_MASK;
            nsf_com1 = LTC2874_NSF2_COM1;
            nsf_com2 = LTC2874_NSF2_COM2;
            nsf_com3 = LTC2874_NSF2_COM3;
            break;
        case 3U:
            slew_mask = LTC2874_MODE2_SLEW3;
            nsf_mask = LTC2874_NSF3_MASK;
            nsf_com1 = LTC2874_NSF3_COM1;
            nsf_com2 = LTC2874_NSF3_COM2;
            nsf_com3 = LTC2874_NSF3_COM3;
            break;
        default:
            slew_mask = LTC2874_MODE2_SLEW4;
            nsf_mask = LTC2874_NSF4_MASK;
            nsf_com1 = LTC2874_NSF4_COM1;
            nsf_com2 = LTC2874_NSF4_COM2;
            nsf_com3 = LTC2874_NSF4_COM3;
            break;
    }

    mode2 &= (uint8_t)~slew_mask;
    nsf &= (uint8_t)~nsf_mask;

    if (baudrate <= 4800U) {
        nsf |= nsf_com1;
    } else if (baudrate <= 38400U) {
        nsf |= nsf_com2;
    } else {
        mode2 |= slew_mask;
        nsf |= nsf_com3;
    }

    LTC2874_SPI_WriteReg(LTC2874_REG_MODE2, mode2);
    LTC2874_SPI_WriteReg(LTC2874_REG_NSF, nsf);
}

static uint8_t IOL_HAL_StartDma(IOL_HalPortContext_t *ctx, uint8_t port_index)
{
    ctx->rx_wrap_count = 0U;
    ctx->rx_read_total = 0U;

    if (HAL_UART_Receive_DMA(ctx->huart, ctx->dma_rx_buf, IOL_RX_RING_SIZE) != HAL_OK) {
        ctx->dma_rx_started = false;

        return 0U;
    }

    ctx->dma_rx_started = true;
    __HAL_UART_CLEAR_IDLEFLAG(ctx->huart);
    __HAL_UART_ENABLE_IT(ctx->huart, UART_IT_IDLE);
    return 1U;
}

void IOL_HAL_RtosInit(void)
{
    if (s_phy_mutex == NULL) {
        s_phy_mutex = xSemaphoreCreateMutex();
        if (s_phy_mutex == NULL) {
            Error_Handler();
        }
    }
}

void IOL_HAL_Init(void)
{
    LTC2874_Init();
    HAL_GPIO_WritePin(TXEN1_GPIO_Port, TXEN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(TXEN2_GPIO_Port, TXEN2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(TXEN3_GPIO_Port, TXEN3_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(TXEN4_GPIO_Port, TXEN4_Pin, GPIO_PIN_RESET);

    /* PHY 自检失败只记录故障，不阻断 H7 状态帧和端口级恢复流程。 */
    (void)LTC2874_Test_SPI_Connection();

    IOL_HAL_PhyLock();
    LTC2874_SPI_WriteReg(LTC2874_REG_MODE1, LTC2874_MODE1_IOLINK_COMPATIBLE);
    LTC2874_SPI_WriteReg(LTC2874_REG_MODE2, LTC2874_MODE2_IOLINK_DEFAULT);
    LTC2874_SPI_WriteReg(LTC2874_REG_NSF, LTC2874_NSF_IOLINK_DEFAULT);
    LTC2874_SPI_WriteReg(LTC2874_REG_ILLM, LTC2874_ILLM_IOLINK_DEFAULT);
    LTC2874_SPI_WriteReg(LTC2874_REG_TMRCTRL, LTC2874_TMRCTRL_IOLINK_DEFAULT);
    IOL_HAL_PhyUnlock();
}

void IOL_HAL_PortPowerOn(uint8_t port_index)
{
    IOL_HalPortContext_t *ctx = IOL_HAL_Port(port_index);

    if (ctx != NULL) {
        IOL_HAL_PhyLock();
        LTC2874_PortPowerOn(ctx->ltc_port);
        IOL_HAL_PhyUnlock();
    }
}

void IOL_HAL_PortPowerOff(uint8_t port_index)
{
    IOL_HalPortContext_t *ctx = IOL_HAL_Port(port_index);

    if (ctx != NULL) {
        IOL_HAL_PhyLock();
        LTC2874_PortPowerOff(ctx->ltc_port);
        IOL_HAL_PhyUnlock();
    }
}

uint8_t IOL_HAL_WakeupPort(uint8_t port_index)
{
    IOL_HalPortContext_t *ctx = IOL_HAL_Port(port_index);

    if (ctx == NULL) {
        return 0U;
    }

    (void)HAL_UART_Abort(ctx->huart);
    (void)HAL_UART_DeInit(ctx->huart);
    ctx->dma_rx_started = false;
    HAL_GPIO_WritePin(ctx->txen_port, ctx->txen_pin, GPIO_PIN_RESET);

    IOL_HAL_PhyLock();
    LTC2874_PortWakeup(ctx->ltc_port);
    IOL_HAL_PhyUnlock();

    vTaskDelay(pdMS_TO_TICKS(1));
    ctx->init_fn();
    return IOL_HAL_StartDma(ctx, port_index);
}

uint8_t IOL_HAL_ReinitPort(uint8_t port_index)
{
    IOL_HalPortContext_t *ctx = IOL_HAL_Port(port_index);
    uint8_t ok;

    if (ctx == NULL) {
        return 0U;
    }

    (void)HAL_UART_Abort(ctx->huart);
    (void)HAL_UART_DeInit(ctx->huart);
    ctx->dma_rx_started = false;
    ctx->reinit_count++;
    ctx->init_fn();
    ok = IOL_HAL_StartDma(ctx, port_index);
    return ok;
}

uint8_t IOL_HAL_SetBaudrate(uint8_t port_index, uint32_t baudrate)
{
    IOL_HalPortContext_t *ctx = IOL_HAL_Port(port_index);

    if (ctx == NULL) {
        return 0U;
    }

    (void)HAL_UART_Abort(ctx->huart);
    __HAL_UART_DISABLE_IT(ctx->huart, UART_IT_IDLE);
    (void)HAL_UART_DeInit(ctx->huart);
    ctx->dma_rx_started = false;

    IOL_HAL_PhyLock();
    IOL_HAL_ApplyPhyBaud(ctx, baudrate);
    IOL_HAL_PhyUnlock();

    ctx->huart->Init.BaudRate = baudrate;
    ctx->huart->Init.WordLength = UART_WORDLENGTH_9B;
    ctx->huart->Init.StopBits = UART_STOPBITS_1;
    ctx->huart->Init.Parity = UART_PARITY_EVEN;
    ctx->huart->Init.Mode = UART_MODE_TX_RX;
    ctx->huart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
    ctx->huart->Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(ctx->huart) != HAL_OK) {
        Error_Handler();
    }

    HAL_GPIO_WritePin(ctx->txen_port, ctx->txen_pin, GPIO_PIN_RESET);
    return IOL_HAL_StartDma(ctx, port_index);
}

uint8_t IOL_HAL_Start_Transaction(uint8_t port_index, const uint8_t *txBuf, uint16_t txLen)
{
    IOL_HalPortContext_t *ctx = IOL_HAL_Port(port_index);

    if ((ctx == NULL) || (txBuf == NULL) || (txLen == 0U)) {
        return IOL_MASTER_ERR_HW;
    }

    if (!ctx->dma_rx_started) {
        if (IOL_HAL_StartDma(ctx, port_index) == 0U) {
            return IOL_MASTER_ERR_HW;
        }
    }

    IOL_HAL_FlushRx(port_index);
    HAL_GPIO_WritePin(ctx->txen_port, ctx->txen_pin, GPIO_PIN_SET);
    __NOP();
    __NOP();
    __NOP();
    __NOP();

    if (HAL_UART_Transmit_IT(ctx->huart, (uint8_t *)txBuf, txLen) != HAL_OK) {
        HAL_GPIO_WritePin(ctx->txen_port, ctx->txen_pin, GPIO_PIN_RESET);

        return IOL_MASTER_ERR_HW;
    }

    return IOL_MASTER_OK;
}

void IOL_HAL_AbortTransaction(uint8_t port_index)
{
    IOL_HalPortContext_t *ctx = IOL_HAL_Port(port_index);

    if (ctx == NULL) {
        return;
    }

    HAL_GPIO_WritePin(ctx->txen_port, ctx->txen_pin, GPIO_PIN_RESET);
    (void)HAL_UART_Abort(ctx->huart);
    __HAL_UART_CLEAR_OREFLAG(ctx->huart);
    __HAL_UART_CLEAR_FEFLAG(ctx->huart);
    __HAL_UART_CLEAR_NEFLAG(ctx->huart);
    __HAL_UART_CLEAR_PEFLAG(ctx->huart);
    ctx->dma_rx_started = false;
    (void)IOL_HAL_StartDma(ctx, port_index);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    uint8_t port_index = IOL_HAL_IndexFromUart(huart);

    if (port_index < IOL_PORT_COUNT) {
        HAL_GPIO_WritePin(s_ports[port_index].txen_port, s_ports[port_index].txen_pin, GPIO_PIN_RESET);
        IOL_MAC_TxCpltCallback(port_index);
    } else if (huart->Instance == USART1) {
        DT35_TxCompleteFromISR();
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uint8_t port_index = IOL_HAL_IndexFromUart(huart);

    if (port_index < IOL_PORT_COUNT) {
        s_ports[port_index].rx_wrap_count++;
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    uint8_t port_index = IOL_HAL_IndexFromUart(huart);

    if (port_index < IOL_PORT_COUNT) {
        HAL_GPIO_WritePin(s_ports[port_index].txen_port, s_ports[port_index].txen_pin, GPIO_PIN_RESET);
        IOL_MAC_ErrorCallback(port_index);
    } else if (huart->Instance == USART1) {
        DT35_TxCompleteFromISR();
    }
}

void IOL_HAL_UART_IRQHandler(UART_HandleTypeDef *huart)
{
    uint8_t port_index;

    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) == RESET) {
        return;
    }

    port_index = IOL_HAL_IndexFromUart(huart);
    if (port_index < IOL_PORT_COUNT) {
        IOL_MAC_RxIdleCallback(port_index, IOL_HAL_GetRxAvailable(port_index));
    }
    __HAL_UART_CLEAR_IDLEFLAG(huart);
}

void IOL_HAL_FlushRx(uint8_t port_index)
{
    IOL_HalPortContext_t *ctx = IOL_HAL_Port(port_index);

    if (ctx != NULL) {
        ctx->rx_read_total = IOL_HAL_GetRxWriteTotal(ctx);
    }
}

uint16_t IOL_HAL_GetRxAvailable(uint8_t port_index)
{
    IOL_HalPortContext_t *ctx = IOL_HAL_Port(port_index);
    uint32_t write_total;
    uint32_t delta;

    if (ctx == NULL) {
        return 0U;
    }

    write_total = IOL_HAL_GetRxWriteTotal(ctx);
    delta = write_total - ctx->rx_read_total;
    if (delta > IOL_RX_RING_SIZE) {
        ctx->dma_overrun_count++;
        ctx->rx_read_total = write_total;

        return 0U;
    }

    return (uint16_t)delta;
}

void IOL_HAL_AdvanceRx(uint8_t port_index, uint16_t len)
{
    IOL_HalPortContext_t *ctx = IOL_HAL_Port(port_index);

    if (ctx != NULL) {
        ctx->rx_read_total += len;
    }
}

void IOL_HAL_ReadRx(uint8_t port_index, uint8_t *dst, uint16_t len)
{
    IOL_HalPortContext_t *ctx = IOL_HAL_Port(port_index);
    uint16_t index;
    uint16_t i;

    if ((ctx == NULL) || (dst == NULL)) {
        return;
    }

    index = (uint16_t)(ctx->rx_read_total % IOL_RX_RING_SIZE);
    for (i = 0U; i < len; i++) {
        dst[i] = ctx->dma_rx_buf[index];
        index = (uint16_t)((index + 1U) % IOL_RX_RING_SIZE);
    }
    ctx->rx_read_total += len;
}

void IOL_HAL_PeekRx(uint8_t port_index, uint8_t *dst, uint16_t len)
{
    IOL_HalPortContext_t *ctx = IOL_HAL_Port(port_index);
    uint16_t index;
    uint16_t i;

    if ((ctx == NULL) || (dst == NULL)) {
        return;
    }

    index = (uint16_t)(ctx->rx_read_total % IOL_RX_RING_SIZE);
    for (i = 0U; i < len; i++) {
        dst[i] = ctx->dma_rx_buf[index];
        index = (uint16_t)((index + 1U) % IOL_RX_RING_SIZE);
    }
}

uint32_t IOL_HAL_GetDmaOverrunCount(uint8_t port_index)
{
    IOL_HalPortContext_t *ctx = IOL_HAL_Port(port_index);
    return (ctx != NULL) ? ctx->dma_overrun_count : 0U;
}

uint32_t IOL_HAL_GetReinitCount(uint8_t port_index)
{
    IOL_HalPortContext_t *ctx = IOL_HAL_Port(port_index);
    return (ctx != NULL) ? ctx->reinit_count : 0U;
}

void IOL_HAL_ReadPhyDiagnostics(IOL_HalPhyDiag_t *out)
{
    if (out == NULL) {
        return;
    }

    IOL_HAL_PhyLock();
    out->irqreg = LTC2874_SPI_ReadReg(LTC2874_REG_IRQREG);
    out->event1 = LTC2874_SPI_ReadReg(LTC2874_REG_EVENT1);
    out->event2 = LTC2874_SPI_ReadReg(LTC2874_REG_EVENT2);
    out->event3 = LTC2874_SPI_ReadReg(LTC2874_REG_EVENT3);
    out->event4 = LTC2874_SPI_ReadReg(LTC2874_REG_EVENT4);
    out->status1 = LTC2874_SPI_ReadReg(LTC2874_REG_STATUS1);
    out->status2 = LTC2874_SPI_ReadReg(LTC2874_REG_STATUS2);
    IOL_HAL_PhyUnlock();
}





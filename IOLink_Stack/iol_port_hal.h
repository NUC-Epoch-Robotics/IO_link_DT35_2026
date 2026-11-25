#ifndef __IOL_PORT_HAL_H
#define __IOL_PORT_HAL_H

#include <stdbool.h>
#include <stdint.h>
#include "ltc2874.h"
#include "main.h"
#include "stm32f4xx_hal.h"
#include "usart.h"

#define IOL_PORT_COUNT       4U
#define IOL_RX_RING_SIZE     256U

typedef struct {
    uint8_t irqreg;
    uint8_t event1;
    uint8_t event2;
    uint8_t event3;
    uint8_t event4;
    uint8_t status1;
    uint8_t status2;
} IOL_HalPhyDiag_t;

void IOL_HAL_Init(void);
void IOL_HAL_RtosInit(void);

void IOL_HAL_PortPowerOn(uint8_t port_index);
void IOL_HAL_PortPowerOff(uint8_t port_index);
uint8_t IOL_HAL_WakeupPort(uint8_t port_index);
uint8_t IOL_HAL_SetBaudrate(uint8_t port_index, uint32_t baudrate);
uint8_t IOL_HAL_ReinitPort(uint8_t port_index);

uint8_t IOL_HAL_Start_Transaction(uint8_t port_index, const uint8_t *txBuf, uint16_t txLen);
void IOL_HAL_AbortTransaction(uint8_t port_index);

uint16_t IOL_HAL_GetRxAvailable(uint8_t port_index);
void IOL_HAL_AdvanceRx(uint8_t port_index, uint16_t len);
void IOL_HAL_ReadRx(uint8_t port_index, uint8_t *dst, uint16_t len);
void IOL_HAL_PeekRx(uint8_t port_index, uint8_t *dst, uint16_t len);
void IOL_HAL_FlushRx(uint8_t port_index);

uint32_t IOL_HAL_GetDmaOverrunCount(uint8_t port_index);
uint32_t IOL_HAL_GetReinitCount(uint8_t port_index);
void IOL_HAL_UART_IRQHandler(UART_HandleTypeDef *huart);
void IOL_HAL_ReadPhyDiagnostics(IOL_HalPhyDiag_t *out);

extern void IOL_MAC_TxCpltCallback(uint8_t port_index);
extern void IOL_MAC_RxIdleCallback(uint8_t port_index, uint16_t rxLength);
extern void IOL_MAC_ErrorCallback(uint8_t port_index);
extern void DT35_TxCompleteFromISR(void);

#endif



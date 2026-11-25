#ifndef _APP_DT35_H_
#define _APP_DT35_H_

#include <stdbool.h>
#include <stdint.h>
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"

#define DT35_PORT_COUNT 4U

typedef struct {
    uint8_t state;
    uint8_t valid;
    uint8_t fault_code;
    uint8_t age_ms;
    uint16_t distance_mm;
    uint16_t sample_seq;
    uint32_t attempts;
    uint32_t good_samples;
    uint32_t timeouts;
    uint32_t checksum_errors;
    uint32_t protocol_errors;
    uint32_t hardware_errors;
    uint32_t dma_overrun_count;
    uint32_t reconnect_count;
    uint32_t tx_busy_count;
    uint32_t schedule_skip_count;
} DT35_PortDiag_t;

uint8_t DT35_Init(void);
void DT35_App_Task(void *argument);
void DT35_1_App_Task(void *argument);
void DT35_2_App_Task(void *argument);
void DT35_3_App_Task(void *argument);
void DT35_4_App_Task(void *argument);
bool IOL_Is_Operate(uint8_t port_index);
void IOL_Master_PortHandler(uint8_t port_index);
void DT35_GetDiagnostics(uint8_t port_index, DT35_PortDiag_t *out);
void DT35_TxCompleteFromISR(void);

extern volatile uint16_t g_dt35_distance[DT35_PORT_COUNT];
extern volatile float g_actu_distance_M[DT35_PORT_COUNT];
extern volatile uint8_t g_dt35_port_enabled[DT35_PORT_COUNT];
extern volatile uint8_t g_dt35_valid_mask;

#endif

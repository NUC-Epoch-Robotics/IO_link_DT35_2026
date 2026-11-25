#include "app_dt35.h"
#include "dt35_protocol.h"
#include "iol_master.h"
#include "iol_port_hal.h"
#include "ltc2874.h"
#include <string.h>

typedef enum {
    PORT_STATE_INACTIVE = 0,
    PORT_STATE_WAIT_POWER_STABLE = 1,
    PORT_STATE_WAKEUP = 2,
    PORT_STATE_WAIT_WAKEUP_RECOVERY = 3,
    PORT_STATE_SET_COM2 = 4,
    PORT_STATE_WAIT_COM2_STABLE = 5,
    PORT_STATE_STARTUP_COM2 = 6,
    PORT_STATE_ENTER_MASTER_IDENT = 7,
    PORT_STATE_ENTER_PREOPERATE = 8,
    PORT_STATE_CONFIGURE = 9,
    PORT_STATE_ENTER_OPERATE = 10,
    PORT_STATE_OPERATE = 11,
    PORT_STATE_RECOVER = 12,
    PORT_STATE_FAULT = 13
} IOL_PortState_t;

typedef enum {
    DT35_FAULT_NONE = 0,
    DT35_FAULT_TIMEOUT = 1,
    DT35_FAULT_CRC = 2,
    DT35_FAULT_PROTO = 3,
    DT35_FAULT_HW = 4,
    DT35_FAULT_DMA = 5,
    DT35_FAULT_CONFIG = 6,
    DT35_FAULT_DISTANCE = 7
} DT35_FaultCode_t;

typedef struct {
    IOL_PortState_t state;
    uint32_t state_entry_tick;
    uint32_t last_read_tick;
    uint32_t last_pd_tick;
    uint8_t failure_count;
    uint8_t good_streak;
    uint8_t fault_code;
    uint8_t valid;
    uint8_t config_step;
    uint16_t distance_mm;
    uint16_t sample_seq;
    uint8_t pd_buffer[2];
    uint32_t attempts;
    uint32_t good_samples;
    uint32_t timeouts;
    uint32_t checksum_errors;
    uint32_t protocol_errors;
    uint32_t hardware_errors;
    uint32_t reconnect_count;
    uint32_t schedule_skip_count;
} DT35_PortContext_t;

static DT35_PortContext_t s_ports[DT35_PORT_COUNT];
static uint8_t s_tx_buffers[2][DT35_FRAME_V2_LEN];
static volatile uint8_t s_tx_busy;
static uint8_t s_tx_active;
static uint16_t s_frame_seq;
static uint32_t s_tx_busy_count;
static uint32_t s_tx_start_tick;

volatile uint16_t g_dt35_distance[DT35_PORT_COUNT] = {0U};
volatile float g_actu_distance_M[DT35_PORT_COUNT] = {0.0f};
volatile uint8_t g_dt35_port_enabled[DT35_PORT_COUNT] = {1U, 1U, 1U, 1U};
volatile uint8_t g_dt35_valid_mask;

IOL_HalPhyDiag_t g_ltc2874_diag;

extern UART_HandleTypeDef huart1;

#define DT35_CYCLE_MS                    10U
#define DT35_STARTUP_SETTLE_MS           10U
#define DT35_FAULT_RETRY_MS              1000U
#define DT35_OPERATE_STABLE_SAMPLES       3U
#define DT35_INVALID_AFTER_FAILURES       3U
#define DT35_REINIT_AFTER_FAILURES        5U
#define DT35_POWER_CYCLE_AFTER_FAILURES  10U
#define DT35_DIAG_POLL_MS                200U

#define DT35_ISDU_PROCESS_DATA_SELECT    83U
#define DT35_ISDU_INTEGRATION_TIME       64U
#define DT35_ISDU_DISTANCE_UNIT          105U
#define DT35_PROCESS_DATA_SELECT_VALUE    3U
#define DT35_INTEGRATION_TIME_VALUE       2U
#define DT35_DISTANCE_UNIT_MM             1U

#define IOL_MCMD_MASTER_IDENT 0x95U
#define IOL_MCMD_PREOPERATE   0x9AU
#define IOL_MCMD_OPERATE      0x99U

static uint8_t DT35_FaultFromStatus(uint8_t status)
{
    switch (status) {
        case IOL_MASTER_TIMEOUT:
            return DT35_FAULT_TIMEOUT;
        case IOL_MASTER_ERR_CKS:
            return DT35_FAULT_CRC;
        case IOL_MASTER_ERR_PROTO:
            return DT35_FAULT_PROTO;
        case IOL_MASTER_ERR_HW:
            return DT35_FAULT_HW;
        case IOL_MASTER_BUSY:
            return DT35_FAULT_HW;
        default:
            return DT35_FAULT_NONE;
    }
}

static void DT35_CountStatus(DT35_PortContext_t *ctx, uint8_t status)
{
    switch (status) {
        case IOL_MASTER_TIMEOUT:
            ctx->timeouts++;
            break;
        case IOL_MASTER_ERR_CKS:
            ctx->checksum_errors++;
            break;
        case IOL_MASTER_ERR_PROTO:
            ctx->protocol_errors++;
            break;
        case IOL_MASTER_ERR_HW:
        case IOL_MASTER_BUSY:
            ctx->hardware_errors++;
            break;
        default:
            break;
    }
}

static void DT35_SetInvalid(DT35_PortContext_t *ctx, uint8_t fault_code)
{
    ctx->valid = 0U;
    ctx->distance_mm = 0xFFFFU;
    ctx->fault_code = fault_code;
}

static void DT35_EnterState(DT35_PortContext_t *ctx, IOL_PortState_t state)
{
    ctx->state = state;
    ctx->state_entry_tick = HAL_GetTick();
    if (state != PORT_STATE_OPERATE) {
        DT35_SetInvalid(ctx, ctx->fault_code);
    }
}

static uint8_t DT35_ReadIsduU8(uint8_t port_index, uint8_t index, uint8_t *value)
{
    uint8_t length = 1U;
    uint8_t data = 0U;

    if ((value == NULL) || (IOL_Master_ISDURead(port_index, index, 0U, false, &data, &length) != IOL_MASTER_OK)) {
        return 0U;
    }
    if (length != 1U) {
        return 0U;
    }
    *value = data;
    return 1U;
}

static uint8_t DT35_WriteIsduU8(uint8_t port_index, uint8_t index, uint8_t value)
{
    return IOL_Master_ISDUWrite(port_index, index, 0U, false, &value, 1U);
}

static uint8_t DT35_ConfigureStep(uint8_t port_index, DT35_PortContext_t *ctx)
{
    uint8_t value;

    switch (ctx->config_step) {
        case 0U:
            (void)DT35_WriteIsduU8(port_index, DT35_ISDU_PROCESS_DATA_SELECT, DT35_PROCESS_DATA_SELECT_VALUE);
            ctx->config_step = 1U;
            return 1U;
        case 1U:
            (void)DT35_ReadIsduU8(port_index, DT35_ISDU_PROCESS_DATA_SELECT, &value);
            ctx->config_step = 2U;
            return 1U;
        case 2U:
            (void)DT35_WriteIsduU8(port_index, DT35_ISDU_DISTANCE_UNIT, DT35_DISTANCE_UNIT_MM);
            ctx->config_step = 3U;
            return 1U;
        case 3U:
            (void)DT35_ReadIsduU8(port_index, DT35_ISDU_DISTANCE_UNIT, &value);
            ctx->config_step = 4U;
            return 1U;
        case 4U:
            (void)DT35_WriteIsduU8(port_index, DT35_ISDU_INTEGRATION_TIME, DT35_INTEGRATION_TIME_VALUE);
            ctx->config_step = 5U;
            return 1U;
        case 5U:
            (void)DT35_ReadIsduU8(port_index, DT35_ISDU_INTEGRATION_TIME, &value);
            ctx->config_step = 6U;
            return 1U;
        default:
            return 0U;
    }
}

static void DT35_HandleFailure(uint8_t port_index, DT35_PortContext_t *ctx, uint8_t status)
{
    uint8_t fault = DT35_FaultFromStatus(status);

    DT35_CountStatus(ctx, status);
    ctx->failure_count++;
    ctx->good_streak = 0U;
    if (fault != DT35_FAULT_NONE) {
        ctx->fault_code = fault;
    }
    if (ctx->failure_count >= DT35_INVALID_AFTER_FAILURES) {
        DT35_SetInvalid(ctx, ctx->fault_code);
    }
    if (ctx->failure_count == DT35_REINIT_AFTER_FAILURES) {
        (void)IOL_HAL_ReinitPort(port_index);
    }
    if (ctx->failure_count >= DT35_POWER_CYCLE_AFTER_FAILURES) {
        ctx->failure_count = 0U;
        ctx->reconnect_count++;
        IOL_HAL_PortPowerOff(port_index);
        vTaskDelay(pdMS_TO_TICKS(10));
        IOL_HAL_PortPowerOn(port_index);
        DT35_EnterState(ctx, PORT_STATE_WAIT_POWER_STABLE);
        ctx->state = PORT_STATE_WAKEUP;
    }
}

static void DT35_UpdateSample(uint8_t port_index, DT35_PortContext_t *ctx)
{
    uint16_t raw_distance = (uint16_t)(((uint16_t)ctx->pd_buffer[0] << 8) | ctx->pd_buffer[1]);

    ctx->failure_count = 0U;
    if (ctx->good_streak < 0xFFU) {
        ctx->good_streak++;
    }

    if ((raw_distance < 50U) || (raw_distance > 50000U) || (raw_distance == 0xFFFFU)) {
        DT35_SetInvalid(ctx, DT35_FAULT_DISTANCE);
        return;
    }

    ctx->distance_mm = raw_distance;
    ctx->last_pd_tick = HAL_GetTick();
    ctx->fault_code = DT35_FAULT_NONE;
    if (ctx->good_streak >= DT35_OPERATE_STABLE_SAMPLES) {
        ctx->valid = 1U;
    }
    if (ctx->valid != 0U) {
        ctx->sample_seq++;
        ctx->good_samples++;
    }

    g_dt35_distance[port_index] = raw_distance;
    g_actu_distance_M[port_index] = (float)raw_distance * 0.001f;
}

bool IOL_Is_Operate(uint8_t port_index)
{
    return (port_index < DT35_PORT_COUNT) && (s_ports[port_index].state == PORT_STATE_OPERATE);
}

void IOL_Master_PortHandler(uint8_t port_index)
{
    DT35_PortContext_t *ctx;
    uint8_t dp_val;
    uint8_t status;

    if (port_index >= DT35_PORT_COUNT) {
        return;
    }
    ctx = &s_ports[port_index];

    switch (ctx->state) {
        case PORT_STATE_INACTIVE:
            IOL_HAL_PortPowerOn(port_index);
            DT35_EnterState(ctx, PORT_STATE_WAIT_POWER_STABLE);
            break;

        case PORT_STATE_WAIT_POWER_STABLE:
            if ((HAL_GetTick() - ctx->state_entry_tick) > 100U) {
                DT35_EnterState(ctx, PORT_STATE_WAKEUP);
            }
            break;

        case PORT_STATE_WAKEUP:
            if (IOL_HAL_WakeupPort(port_index) == 0U) {
                ctx->fault_code = DT35_FAULT_HW;
                DT35_EnterState(ctx, PORT_STATE_FAULT);
                break;
            }
            DT35_EnterState(ctx, PORT_STATE_WAIT_WAKEUP_RECOVERY);
            break;

        case PORT_STATE_WAIT_WAKEUP_RECOVERY:
            if ((HAL_GetTick() - ctx->state_entry_tick) > 5U) {
                DT35_EnterState(ctx, PORT_STATE_SET_COM2);
            }
            break;

        case PORT_STATE_SET_COM2:
            if (IOL_HAL_SetBaudrate(port_index, 38400U) == 0U) {
                ctx->fault_code = DT35_FAULT_HW;
                DT35_EnterState(ctx, PORT_STATE_FAULT);
                break;
            }
            DT35_EnterState(ctx, PORT_STATE_WAIT_COM2_STABLE);
            break;

        case PORT_STATE_WAIT_COM2_STABLE:
            if ((HAL_GetTick() - ctx->state_entry_tick) >= DT35_STARTUP_SETTLE_MS) {
                DT35_EnterState(ctx, PORT_STATE_STARTUP_COM2);
            }
            break;

        case PORT_STATE_STARTUP_COM2:
            status = IOL_Master_ReadDirectParameter(port_index, 0x02U, &dp_val);
            if (status == IOL_MASTER_OK) {
                DT35_EnterState(ctx, PORT_STATE_ENTER_MASTER_IDENT);
            } else {
                ctx->fault_code = DT35_FaultFromStatus(status);
                DT35_EnterState(ctx, PORT_STATE_FAULT);
            }
            break;

        case PORT_STATE_ENTER_MASTER_IDENT:
            status = IOL_Master_WriteDirectParameter(port_index, 0x00U, IOL_MCMD_MASTER_IDENT);
            if (status == IOL_MASTER_OK) {
                DT35_EnterState(ctx, PORT_STATE_ENTER_PREOPERATE);
            } else {
                ctx->fault_code = DT35_FaultFromStatus(status);
                DT35_EnterState(ctx, PORT_STATE_FAULT);
            }
            break;

        case PORT_STATE_ENTER_PREOPERATE:
            status = IOL_Master_WriteDirectParameter(port_index, 0x00U, IOL_MCMD_PREOPERATE);
            if (status == IOL_MASTER_OK) {
                ctx->config_step = 0U;
                DT35_EnterState(ctx, PORT_STATE_CONFIGURE);
            } else {
                ctx->fault_code = DT35_FaultFromStatus(status);
                DT35_EnterState(ctx, PORT_STATE_FAULT);
            }
            break;

        case PORT_STATE_CONFIGURE:
            if (DT35_ConfigureStep(port_index, ctx) == 0U) {
                ctx->fault_code = DT35_FAULT_CONFIG;
                DT35_EnterState(ctx, PORT_STATE_FAULT);
            } else if (ctx->config_step >= 6U) {
                DT35_EnterState(ctx, PORT_STATE_ENTER_OPERATE);
            }
            break;

        case PORT_STATE_ENTER_OPERATE:
            status = IOL_Master_WriteDirectParameter(port_index, 0x00U, IOL_MCMD_OPERATE);
            if (status == IOL_MASTER_OK) {
                ctx->last_read_tick = HAL_GetTick();
                ctx->last_pd_tick = HAL_GetTick();
                ctx->failure_count = 0U;
                ctx->good_streak = 0U;
                ctx->fault_code = DT35_FAULT_NONE;
                DT35_EnterState(ctx, PORT_STATE_OPERATE);
            } else {
                ctx->fault_code = DT35_FaultFromStatus(status);
                DT35_EnterState(ctx, PORT_STATE_FAULT);
            }
            break;

        case PORT_STATE_OPERATE:
            break;

        case PORT_STATE_RECOVER:
            DT35_EnterState(ctx, PORT_STATE_WAKEUP);
            break;

        case PORT_STATE_FAULT:
            DT35_SetInvalid(ctx, ctx->fault_code);
            if ((HAL_GetTick() - ctx->state_entry_tick) > DT35_FAULT_RETRY_MS) {
                ctx->reconnect_count++;
                DT35_EnterState(ctx, PORT_STATE_WAKEUP);
            }
            break;

        default:
            DT35_EnterState(ctx, PORT_STATE_FAULT);
            break;
    }
}

static void DT35_Port_Task_Run(uint8_t port_index)
{
    DT35_PortContext_t *ctx;
    TickType_t last_wake;
    uint32_t now;

    if (port_index >= DT35_PORT_COUNT) {
        vTaskDelete(NULL);
        return;
    }

    ctx = &s_ports[port_index];
    last_wake = xTaskGetTickCount();

    for (;;) {
        if (g_dt35_port_enabled[port_index] != 0U) {
            if (ctx->state == PORT_STATE_OPERATE) {
                uint8_t status;

                now = HAL_GetTick();
                if ((now - ctx->last_read_tick) >= DT35_CYCLE_MS) {
                    ctx->last_read_tick = now;
                    ctx->attempts++;
                    status = IOL_Master_ReadProcessData(port_index, ctx->pd_buffer, 2U);
                    if (status == IOL_MASTER_OK) {
                        DT35_UpdateSample(port_index, ctx);
                    } else {
                        DT35_HandleFailure(port_index, ctx, status);
                    }
                }
            } else {
                IOL_Master_PortHandler(port_index);
            }
        }

        {
            TickType_t now_ticks = xTaskGetTickCount();
            if ((now_ticks - last_wake) >= pdMS_TO_TICKS(DT35_CYCLE_MS)) {
                ctx->schedule_skip_count++;
                last_wake = now_ticks;
            }
        }
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(DT35_CYCLE_MS));
    }
}

void DT35_TxCompleteFromISR(void)
{
    s_tx_busy = 0U;
}

void DT35_App_Task(void *argument)
{
    TickType_t last_wake;
    uint32_t last_diag_tick = 0U;
    uint8_t status[4];
    uint8_t age[4];
    uint16_t distance[4];
    uint16_t sample_seq[4];
    uint8_t valid_mask;
    uint8_t tx_index;
    uint8_t i;
    uint16_t next_frame_seq;

    (void)argument;
    last_wake = xTaskGetTickCount();

    for (;;) {
        uint32_t now = HAL_GetTick();

        if ((now - last_diag_tick) >= DT35_DIAG_POLL_MS) {
            last_diag_tick = now;
            IOL_HAL_ReadPhyDiagnostics(&g_ltc2874_diag);
        }

        if ((s_tx_busy != 0U) && ((now - s_tx_start_tick) > 5U)) {
            (void)HAL_UART_AbortTransmit(&huart1);
            s_tx_busy = 0U;
            s_tx_busy_count++;
        }

        if (s_tx_busy == 0U) {
            valid_mask = 0U;
            taskENTER_CRITICAL();
            for (i = 0U; i < DT35_PORT_COUNT; i++) {
                DT35_PortContext_t *ctx = &s_ports[i];
                uint8_t is_valid = (uint8_t)((ctx->valid != 0U) && (ctx->state == PORT_STATE_OPERATE));
                status[i] = (uint8_t)(((uint8_t)ctx->state << 4) | (ctx->fault_code & 0x0FU));
                if (is_valid != 0U) {
                    uint32_t sample_age = now - ctx->last_pd_tick;
                    age[i] = (sample_age > 255U) ? 255U : (uint8_t)sample_age;
                    distance[i] = ctx->distance_mm;
                    sample_seq[i] = ctx->sample_seq;
                    valid_mask |= (uint8_t)(1U << i);
                } else {
                    age[i] = 255U;
                    distance[i] = 0xFFFFU;
                    sample_seq[i] = ctx->sample_seq;
                }
            }
            taskEXIT_CRITICAL();

            g_dt35_valid_mask = valid_mask;
            tx_index = (uint8_t)(s_tx_active ^ 1U);
            next_frame_seq = (uint16_t)(s_frame_seq + 1U);
            DT35_BuildStatusFrame(s_tx_buffers[tx_index], next_frame_seq, valid_mask,
                                  status, age, distance, sample_seq);

            if (HAL_UART_Transmit_DMA(&huart1, s_tx_buffers[tx_index], DT35_FRAME_V2_LEN) == HAL_OK) {
                s_tx_active = tx_index;
                s_frame_seq = next_frame_seq;
                s_tx_start_tick = now;
                s_tx_busy = 1U;
            } else {
                s_tx_busy_count++;
            }
        } else {
            s_tx_busy_count++;
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(DT35_CYCLE_MS));
    }
}

void DT35_1_App_Task(void *argument)
{
    (void)argument;
    DT35_Port_Task_Run(0U);
}

void DT35_2_App_Task(void *argument)
{
    (void)argument;
    DT35_Port_Task_Run(1U);
}

void DT35_3_App_Task(void *argument)
{
    (void)argument;
    DT35_Port_Task_Run(2U);
}

void DT35_4_App_Task(void *argument)
{
    (void)argument;
    DT35_Port_Task_Run(3U);
}

uint8_t DT35_Init(void)
{
    uint8_t i;

    for (i = 0U; i < DT35_PORT_COUNT; i++) {
        memset(&s_ports[i], 0, sizeof(s_ports[i]));
        s_ports[i].state = PORT_STATE_INACTIVE;
        s_ports[i].distance_mm = 0xFFFFU;
        s_ports[i].fault_code = DT35_FAULT_NONE;
    }
    g_dt35_valid_mask = 0U;
    return IOL_MASTER_OK;
}

void DT35_GetDiagnostics(uint8_t port_index, DT35_PortDiag_t *out)
{
    DT35_PortContext_t *ctx;

    if ((port_index >= DT35_PORT_COUNT) || (out == NULL)) {
        return;
    }
    ctx = &s_ports[port_index];
    out->state = (uint8_t)ctx->state;
    out->valid = ctx->valid;
    out->fault_code = ctx->fault_code;
    out->distance_mm = ctx->distance_mm;
    out->sample_seq = ctx->sample_seq;
    out->age_ms = (ctx->valid != 0U) ? (uint8_t)(HAL_GetTick() - ctx->last_pd_tick) : 255U;
    out->attempts = ctx->attempts;
    out->good_samples = ctx->good_samples;
    out->timeouts = ctx->timeouts;
    out->checksum_errors = ctx->checksum_errors;
    out->protocol_errors = ctx->protocol_errors;
    out->hardware_errors = ctx->hardware_errors;
    out->dma_overrun_count = IOL_HAL_GetDmaOverrunCount(port_index);
    out->reconnect_count = ctx->reconnect_count;
    out->tx_busy_count = s_tx_busy_count;
    out->schedule_skip_count = ctx->schedule_skip_count;
}


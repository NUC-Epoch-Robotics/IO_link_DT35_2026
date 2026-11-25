#include "Ranging_rx.h"
#include "usart.h"
#include <string.h>
#include <stdint.h>

// --- 自定义协议宏定义 ---
#define RX_HEADER1    0xFF
#define RX_HEADER2    0x55
#define PAYLOAD_FLOAT_COUNT 4
#define PAYLOAD_BYTE_LEN   (PAYLOAD_FLOAT_COUNT * 4) // 16字节

extern UART_HandleTypeDef huart4; 

// 接收状态机枚举
typedef enum {
    STATE_H1 = 0,
    STATE_H2,
    STATE_COMMAND_H,
    STATE_COMMAND_L,
    STATE_LEN,
    STATE_PAYLOAD
} Ranging_State_e;

// 状态机变量
static Ranging_State_e rx_state = STATE_H1;
static uint8_t rx_buf[PAYLOAD_BYTE_LEN];
static uint8_t rx_idx = 0;
static uint16_t curr_command = 0;
static uint8_t expected_len = 0;

// DMA 环形缓冲区
#define RANGING_DMA_RX_LEN 128
uint8_t ranging_dma_rx_buf[RANGING_DMA_RX_LEN];

/* ========================================================
 * 私有函数：解析接收到的测距/浮点数据
 * ======================================================== */
static void Parse_Ranging_Frame(uint16_t cmd, uint8_t *payload, float *val) {

    // 定义联合体用于转换
    union {
        uint8_t bytes[PAYLOAD_BYTE_LEN];
        float values[PAYLOAD_FLOAT_COUNT];
    } converter;

    // 拷贝数据
    memcpy(converter.bytes, payload, PAYLOAD_BYTE_LEN);

    // 提取 float 数据
    for (uint8_t i= 0; i < 4; i++)
    {
        val[i] = converter.values[i];
    }

    // TODO: 根据 cmd (命令号) 处理数据
    // 例如：Update_Ranging_System(cmd, val0, val1);
}
float val[4];
static float latest_ranging_val[4] = {0};
static uint32_t latest_ranging_tick = 0;
static uint8_t latest_ranging_valid = 0;
/* ========================================================
 * 字节解码状态机
 * ======================================================== */
void Ranging_Decode_Byte(uint8_t byte) {
    
    switch (rx_state) {
        case STATE_H1:
            if (byte == RX_HEADER1) rx_state = STATE_H2;
            break;

        case STATE_H2:
            if (byte == RX_HEADER2) rx_state = STATE_COMMAND_H;
            else rx_state = STATE_H1;
            break;

        case STATE_COMMAND_H:
            curr_command = (uint16_t)byte << 8;
            rx_state = STATE_COMMAND_L;
            break;
        case STATE_COMMAND_L:
            curr_command |= byte;
            rx_state = STATE_LEN;
            break;

        case STATE_LEN:
            expected_len = byte;
            // 如果长度符合预期（4个float=16字节），进入载荷接收
            if (expected_len == PAYLOAD_BYTE_LEN) {
                rx_idx = 0;
                rx_state = STATE_PAYLOAD;
            } else {
                rx_state = STATE_H1; // 长度不匹配则丢弃
            }
            break;

        case STATE_PAYLOAD:
            rx_buf[rx_idx++] = byte;
            if (rx_idx >= PAYLOAD_BYTE_LEN) {
                Parse_Ranging_Frame(curr_command, rx_buf,val);
                memcpy(latest_ranging_val, val, sizeof(latest_ranging_val));
                latest_ranging_tick = HAL_GetTick();
                latest_ranging_valid = 1;
                DT35_Execute_Fusion(val);
                rx_state = STATE_H1; // 解析完成，回到初始态
            }
            break;

        default:
            rx_state = STATE_H1;
            break;
    }
}

/* ========================================================
 * 测距数据接收任务
 * ======================================================== */
void Ranging_Task(void *argument) {
    // 开启 DMA 接收
    HAL_UART_Receive_DMA(&huart4, ranging_dma_rx_buf, RANGING_DMA_RX_LEN);

    uint16_t read_idx = 0;

    for (;;) {
        // 获取 DMA 当前写指针位置
        uint16_t write_idx = RANGING_DMA_RX_LEN - __HAL_DMA_GET_COUNTER(huart4.hdmarx);

        // 处理新接收到的字节 
        while (read_idx != write_idx) {
            Ranging_Decode_Byte(ranging_dma_rx_buf[read_idx]);
            read_idx = (read_idx + 1) % RANGING_DMA_RX_LEN;
        }

        // 任务延时，根据发送频率调整，通常 1-5ms 即可
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

uint8_t Ranging_Get_Latest(float out[4], uint32_t *tick)
{
    if (!latest_ranging_valid) {
        return 0;
    }

    memcpy(out, latest_ranging_val, sizeof(latest_ranging_val));
    if (tick != NULL) {
        *tick = latest_ranging_tick;
    }
    return 1;
}

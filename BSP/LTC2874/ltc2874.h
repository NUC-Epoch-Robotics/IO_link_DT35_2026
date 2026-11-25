#ifndef __LTC2874_H
#define __LTC2874_H

#include "stm32f4xx_hal.h"

extern SPI_HandleTypeDef hspi1;

typedef enum
{
    LTC2874_DIAG_NOT_RUN = 0,
    LTC2874_DIAG_INIT_DELAY_DONE,
    LTC2874_DIAG_READ_DEFAULTS_OK,
    LTC2874_DIAG_WRITEBACK_OK,
    LTC2874_DIAG_SUCCESS,
    LTC2874_DIAG_FAIL_READ_DEFAULTS,
    LTC2874_DIAG_FAIL_WRITEBACK
} LTC2874_DiagState_t;

extern volatile LTC2874_DiagState_t g_ltc2874_diag_state;
extern volatile uint8_t g_ltc2874_diag_raw_reads[8];

#define LTC2874_REG_IRQREG     0x00U
#define LTC2874_REG_IRQMASK    0x01U
#define LTC2874_REG_EVENT1     0x02U
#define LTC2874_REG_EVENT2     0x03U
#define LTC2874_REG_EVENT3     0x04U
#define LTC2874_REG_EVENT4     0x05U
#define LTC2874_REG_STATUS1    0x06U
#define LTC2874_REG_STATUS2    0x07U
#define LTC2874_REG_MODE1      0x08U
#define LTC2874_REG_MODE2      0x09U
#define LTC2874_REG_NSF        0x0AU
#define LTC2874_REG_ILLM       0x0BU
#define LTC2874_REG_TMRCTRL    0x0CU
#define LTC2874_REG_CTRL1      0x0DU
#define LTC2874_REG_CTRL2      0x0EU

#define LTC2874_MODE1_IOLINK_COMPATIBLE 0x8FU
#define LTC2874_MODE2_IOLINK_DEFAULT    0xF4U
#define LTC2874_NSF_IOLINK_DEFAULT      0xFFU
#define LTC2874_ILLM_IOLINK_DEFAULT     0xFFU
#define LTC2874_TMRCTRL_IOLINK_DEFAULT  0x85U

#define LTC2874_CTRL1_WKUP1             0x10U
#define LTC2874_CTRL1_WKUP2             0x20U
#define LTC2874_CTRL1_WKUP3             0x40U
#define LTC2874_CTRL1_WKUP4             0x80U
#define LTC2874_CTRL2_ENL1              0x10U
#define LTC2874_CTRL2_ENL2              0x20U
#define LTC2874_CTRL2_ENL3              0x40U
#define LTC2874_CTRL2_ENL4              0x80U

#define LTC2874_MODE2_SLEW1             0x10U
#define LTC2874_MODE2_SLEW2             0x20U
#define LTC2874_MODE2_SLEW3             0x40U
#define LTC2874_MODE2_SLEW4             0x80U
#define LTC2874_NSF1_MASK               0x03U
#define LTC2874_NSF1_DISABLED           0x00U
#define LTC2874_NSF1_COM1               0x01U
#define LTC2874_NSF1_COM2               0x02U
#define LTC2874_NSF1_COM3               0x03U
#define LTC2874_NSF2_MASK               0x0CU
#define LTC2874_NSF2_COM1               0x04U
#define LTC2874_NSF2_COM2               0x08U
#define LTC2874_NSF2_COM3               0x0CU
#define LTC2874_NSF3_MASK               0x30U
#define LTC2874_NSF3_COM1               0x10U
#define LTC2874_NSF3_COM2               0x20U
#define LTC2874_NSF3_COM3               0x30U
#define LTC2874_NSF4_MASK               0xC0U
#define LTC2874_NSF4_COM1               0x40U
#define LTC2874_NSF4_COM2               0x80U
#define LTC2874_NSF4_COM3               0xC0U

void LTC2874_Init(void);
void LTC2874_SPI_WriteReg(uint8_t regAddr, uint8_t data);
uint8_t LTC2874_SPI_ReadReg(uint8_t regAddr);
void LTC2874_PortPowerOn(uint8_t port);
void LTC2874_PortPowerOff(uint8_t port);
void LTC2874_PortWakeup(uint8_t port);
void LTC2874_Port1_PowerOn(void);
void LTC2874_Port1_PowerOff(void);
void LTC2874_Port1_Wakeup(void);
uint8_t LTC2874_Test_SPI_Connection(void);
uint8_t LTC2874_Test_SPI_ReadDefaults(void);

#endif

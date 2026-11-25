#include "ltc2874.h"

#define LTC2874_CMD_READ      0x00U
#define LTC2874_CMD_WRITE     0x01U
#define LTC2874_CMD_UPDATE    0x02U
#define LTC2874_CMD_WRTUPD    0x03U
#define LTC2874_CMD_RESET     0x07U

volatile LTC2874_DiagState_t g_ltc2874_diag_state = LTC2874_DIAG_NOT_RUN;
volatile uint8_t g_ltc2874_diag_raw_reads[8] = {0U};

static uint8_t LTC2874_BuildCmdByte(uint8_t cmd, uint8_t regAddr)
{
    return (uint8_t)((cmd << 5) | ((regAddr & 0x0FU) << 1));
}

static void LTC2874_SendReset(void)
{
    uint8_t txBuffer[2];

    txBuffer[0] = LTC2874_BuildCmdByte(LTC2874_CMD_RESET, 0x00U);
    txBuffer[1] = 0x00U;

    HAL_SPI_Transmit(&hspi1, txBuffer, 2, 5);
}

void LTC2874_Init(void)
{
    HAL_Delay(100);
    g_ltc2874_diag_state = LTC2874_DIAG_INIT_DELAY_DONE;
}

void LTC2874_SPI_WriteReg(uint8_t regAddr, uint8_t data)
{
    uint8_t txBuffer[2];

    txBuffer[0] = LTC2874_BuildCmdByte(LTC2874_CMD_WRTUPD, regAddr);
    txBuffer[1] = data;

    HAL_SPI_Transmit(&hspi1, txBuffer, 2, 5);
}

uint8_t LTC2874_SPI_ReadReg(uint8_t regAddr)
{
    uint8_t txBuffer[2];
    uint8_t rxBuffer[2] = {0U, 0U};

    txBuffer[0] = LTC2874_BuildCmdByte(LTC2874_CMD_READ, regAddr);
    txBuffer[1] = 0x00U;

    HAL_SPI_TransmitReceive(&hspi1, txBuffer, rxBuffer, 2, 5);

    return rxBuffer[1];
}

void LTC2874_Port1_PowerOn(void)
{
    LTC2874_PortPowerOn(1U);
}

void LTC2874_Port1_PowerOff(void)
{
    LTC2874_PortPowerOff(1U);
}

void LTC2874_Port1_Wakeup(void)
{
    LTC2874_PortWakeup(1U);
}

void LTC2874_PortPowerOn(uint8_t port)
{
    uint8_t currentVal = LTC2874_SPI_ReadReg(LTC2874_REG_CTRL2);
    uint8_t mask = LTC2874_CTRL2_ENL1;

    switch (port)
    {
        case 2U: mask = LTC2874_CTRL2_ENL2; break;
        case 3U: mask = LTC2874_CTRL2_ENL3; break;
        case 4U: mask = LTC2874_CTRL2_ENL4; break;
        default: mask = LTC2874_CTRL2_ENL1; break;
    }

    currentVal |= mask;
    LTC2874_SPI_WriteReg(LTC2874_REG_CTRL2, currentVal);
}

void LTC2874_PortPowerOff(uint8_t port)
{
    uint8_t currentVal = LTC2874_SPI_ReadReg(LTC2874_REG_CTRL2);
    uint8_t mask = LTC2874_CTRL2_ENL1;

    switch (port)
    {
        case 2U: mask = LTC2874_CTRL2_ENL2; break;
        case 3U: mask = LTC2874_CTRL2_ENL3; break;
        case 4U: mask = LTC2874_CTRL2_ENL4; break;
        default: mask = LTC2874_CTRL2_ENL1; break;
    }

    currentVal &= (uint8_t)~mask;
    LTC2874_SPI_WriteReg(LTC2874_REG_CTRL2, currentVal);
}

void LTC2874_PortWakeup(uint8_t port)
{
    uint8_t mask = LTC2874_CTRL1_WKUP1;

    switch (port)
    {
        case 2U: mask = LTC2874_CTRL1_WKUP2; break;
        case 3U: mask = LTC2874_CTRL1_WKUP3; break;
        case 4U: mask = LTC2874_CTRL1_WKUP4; break;
        default: mask = LTC2874_CTRL1_WKUP1; break;
    }

    LTC2874_SPI_WriteReg(LTC2874_REG_CTRL1, mask);
}

uint8_t LTC2874_Test_SPI_ReadDefaults(void)
{
    static const uint8_t expected[7] =
    {
        0x00U,
        0xFEU,
        0x8FU,
        0xF4U,
        0xFFU,
        0xFFU,
        0x85U
    };
    static const uint8_t regs[7] =
    {
        LTC2874_REG_IRQREG,
        LTC2874_REG_IRQMASK,
        LTC2874_REG_MODE1,
        LTC2874_REG_MODE2,
        LTC2874_REG_NSF,
        LTC2874_REG_ILLM,
        LTC2874_REG_TMRCTRL
    };
    uint8_t idx;
    uint8_t ok = 1U;

    LTC2874_SendReset();

    for (idx = 0U; idx < 7U; ++idx)
    {
        g_ltc2874_diag_raw_reads[idx] = LTC2874_SPI_ReadReg(regs[idx]);
        if (g_ltc2874_diag_raw_reads[idx] != expected[idx])
        {
            ok = 0U;
        }
    }

    g_ltc2874_diag_state = (ok != 0U) ? LTC2874_DIAG_READ_DEFAULTS_OK : LTC2874_DIAG_FAIL_READ_DEFAULTS;
    return ok;
}

uint8_t LTC2874_Test_SPI_Connection(void)
{
    uint8_t originVal;
    uint8_t toggledVal;
    uint8_t testVal;

    if (LTC2874_Test_SPI_ReadDefaults() == 0U)
    {
        return 0U;
    }

    LTC2874_SendReset();

    originVal = LTC2874_SPI_ReadReg(LTC2874_REG_IRQMASK);
    g_ltc2874_diag_raw_reads[7] = originVal;
    if (originVal != 0xFEU)
    {
        g_ltc2874_diag_state = LTC2874_DIAG_FAIL_WRITEBACK;
        return 0U;
    }

    toggledVal = (uint8_t)(originVal ^ 0x01U);
    LTC2874_SPI_WriteReg(LTC2874_REG_IRQMASK, toggledVal);

    testVal = LTC2874_SPI_ReadReg(LTC2874_REG_IRQMASK);
    LTC2874_SPI_WriteReg(LTC2874_REG_IRQMASK, originVal);

    if (testVal != toggledVal)
    {
        g_ltc2874_diag_state = LTC2874_DIAG_FAIL_WRITEBACK;
        return 0U;
    }

    g_ltc2874_diag_state = LTC2874_DIAG_WRITEBACK_OK;
    g_ltc2874_diag_state = LTC2874_DIAG_SUCCESS;
    return 1U;
}

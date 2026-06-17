#include "wit_c_sdk.h"

static SerialWrite p_WitSerialWriteFunc = NULL;
static RegUpdateCb p_WitRegUpdateCbFunc = NULL;
static DelaymsCb p_WitDelaymsFunc = NULL;

static uint8_t s_ucAddr = 0xff;
static uint8_t s_ucWitDataBuff[WIT_DATA_BUFF_SIZE];
static uint32_t s_uiWitDataCnt = 0, s_uiProtoclo = 0;
int16_t sReg[REGSIZE];

static uint8_t CalSum(uint8_t *data, uint32_t len)
{
    uint8_t sum = 0;
    for (uint32_t i = 0; i < len; i++) sum += data[i];
    return sum;
}

static void CopeWitData(uint8_t ucIndex, uint16_t *p_data, uint32_t uiLen)
{
    uint32_t uiReg1 = 0, uiReg1Len = 0;
    switch (ucIndex) {
        case WIT_ACC:   uiReg1 = AX;    uiReg1Len = 3;  break;
        case WIT_ANGLE: uiReg1 = Roll;  uiReg1Len = 3;  break;
        case WIT_GYRO:  uiReg1 = GX;    uiReg1Len = 3;  break;
        case WIT_TIME:  uiReg1 = YYMM;  uiReg1Len = 4;  break;
        default: return;
    }
    for (uint32_t i = 0; i < uiReg1Len && i < uiLen; i++) {
        sReg[uiReg1 + i] = p_data[i];
    }
    if (p_WitRegUpdateCbFunc) p_WitRegUpdateCbFunc(uiReg1, uiReg1Len);
}

void WitSerialDataIn(uint8_t ucData)
{
    if (!p_WitRegUpdateCbFunc) return;
    s_ucWitDataBuff[s_uiWitDataCnt++] = ucData;

    if (s_ucWitDataBuff[0] != 0x55) {
        s_uiWitDataCnt--;
        if (s_uiWitDataCnt)
            memmove(s_ucWitDataBuff, &s_ucWitDataBuff[1], s_uiWitDataCnt);
        return;
    }
    if (s_uiWitDataCnt >= 11) {
        uint8_t ucSum = CalSum(s_ucWitDataBuff, 10);
        if (ucSum == s_ucWitDataBuff[10]) {
            uint16_t usData[4];
            usData[0] = ((uint16_t)s_ucWitDataBuff[3] << 8) | s_ucWitDataBuff[2];
            usData[1] = ((uint16_t)s_ucWitDataBuff[5] << 8) | s_ucWitDataBuff[4];
            usData[2] = ((uint16_t)s_ucWitDataBuff[7] << 8) | s_ucWitDataBuff[6];
            usData[3] = ((uint16_t)s_ucWitDataBuff[9] << 8) | s_ucWitDataBuff[8];
            CopeWitData(s_ucWitDataBuff[1], usData, 4);
        } else {
            s_uiWitDataCnt--;
            memmove(s_ucWitDataBuff, &s_ucWitDataBuff[1], s_uiWitDataCnt);
            return;
        }
        s_uiWitDataCnt = 0;
    }
    if (s_uiWitDataCnt == WIT_DATA_BUFF_SIZE) s_uiWitDataCnt = 0;
}

int32_t WitSerialWriteRegister(SerialWrite Write_func)
{
    if (!Write_func) return WIT_HAL_INVAL;
    p_WitSerialWriteFunc = Write_func;
    return WIT_HAL_OK;
}

int32_t WitRegisterCallBack(RegUpdateCb update_func)
{
    if (!update_func) return WIT_HAL_INVAL;
    p_WitRegUpdateCbFunc = update_func;
    return WIT_HAL_OK;
}

int32_t WitDelayMsRegister(DelaymsCb delayms_func)
{
    if (!delayms_func) return WIT_HAL_INVAL;
    p_WitDelaymsFunc = delayms_func;
    return WIT_HAL_OK;
}

int32_t WitInit(uint32_t uiProtocol, uint8_t ucAddr)
{
    s_uiProtoclo = uiProtocol;
    s_ucAddr = ucAddr;
    s_uiWitDataCnt = 0;
    return WIT_HAL_OK;
}

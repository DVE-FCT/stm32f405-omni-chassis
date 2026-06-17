/**
 * @file    wit_gyro.c
 * @brief   维特 6 轴陀螺仪驱动 — UART 协议 (WIT_PROTOCOL_NORMAL)
 *
 * 硬件: USART2 (PA2 TX, PA3 RX), 波特率需与传感器匹配 (默认 115200)
 * 协议: 帧头 0x55 + 11字节数据帧 + 校验和
 *
 * 使用:
 *   1. GYR_Init() → 注册回调
 *   2. USART2 RX 中断 → GYR_FeedByte(byte)
 *   3. main 循环 → GYR_Updata() → 读取 fAngle/fAcc/fGyro
 */

#include "wit_gyro.h"

/* 全局 */
float fAngle[3], fAcc[3], fGyro[3];
static volatile uint8_t s_cDataUpdate = 0;

/* ---- 回调: 传感器数据到达 ---- */
static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum)
{
    for (uint32_t i = 0; i < uiRegNum; i++) {
        switch (uiReg + i) {
            case Roll:
            case Pitch:
            case Yaw:
                s_cDataUpdate |= ANGLE_UPDATE;
                break;
            case AX:
            case AY:
            case AZ:
                s_cDataUpdate |= ACC_UPDATE;
                break;
            case GX:
            case GY:
            case GZ:
                s_cDataUpdate |= GYRO_UPDATE;
                break;
            default: break;
        }
    }
}

/* ---- 延时回调 ---- */
static void Delayms(uint16_t ucMs)
{
    HAL_Delay(ucMs);
}

/* ---- 初始化 ---- */
void GYR_Init(void)
{
    WitInit(WIT_PROTOCOL_NORMAL, 0x50);
    WitRegisterCallBack(SensorDataUpdata);
    WitDelayMsRegister(Delayms);
}

/* ---- 喂 RX 字节 ---- */
void GYR_FeedByte(uint8_t byte)
{
    WitSerialDataIn(byte);
}

/* ---- 拉取最新数据 ---- */
void GYR_Updata(void)
{
    if (s_cDataUpdate & ACC_UPDATE) {
        for (int i = 0; i < 3; i++)
            fAcc[i] = sReg[AX + i] / 32768.0f * 16.0f;
        s_cDataUpdate &= ~ACC_UPDATE;
    }
    if (s_cDataUpdate & GYRO_UPDATE) {
        for (int i = 0; i < 3; i++)
            fGyro[i] = sReg[GX + i] / 32768.0f * 2000.0f;
        s_cDataUpdate &= ~GYRO_UPDATE;
    }
    if (s_cDataUpdate & ANGLE_UPDATE) {
        for (int i = 0; i < 3; i++)
            fAngle[i] = sReg[Roll + i] / 32768.0f * 180.0f;
        s_cDataUpdate &= ~ANGLE_UPDATE;
    }
}

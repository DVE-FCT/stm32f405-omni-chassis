#ifndef __WIT_C_SDK_H
#define __WIT_C_SDK_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define WIT_HAL_OK      (0)
#define WIT_HAL_BUSY    (-1)
#define WIT_HAL_TIMEOUT (-2)
#define WIT_HAL_ERROR   (-3)
#define WIT_HAL_NOMEM   (-4)
#define WIT_HAL_EMPTY   (-5)
#define WIT_HAL_INVAL   (-6)

#define WIT_DATA_BUFF_SIZE  256

#define WIT_PROTOCOL_NORMAL 0
#define WIT_PROTOCOL_MODBUS 1

#define REGSIZE 0x90

/* Register addresses */
#define AX      0x34
#define AY      0x35
#define AZ      0x36
#define GX      0x37
#define GY      0x38
#define GZ      0x39
#define Roll    0x3d
#define Pitch   0x3e
#define Yaw     0x3f
#define TEMP    0x40
#define VERSION 0x2e
#define YYMM    0x30

/* Serial output types */
#define WIT_TIME    0x50
#define WIT_ACC     0x51
#define WIT_GYRO    0x52
#define WIT_ANGLE   0x53

/* serial function */
typedef void (*SerialWrite)(uint8_t *p_ucData, uint32_t uiLen);
int32_t WitSerialWriteRegister(SerialWrite write_func);
void WitSerialDataIn(uint8_t ucData);

/* Delay function */
typedef void (*DelaymsCb)(uint16_t ucMs);
int32_t WitDelayMsRegister(DelaymsCb delayms_func);

/* Callback */
typedef void (*RegUpdateCb)(uint32_t uiReg, uint32_t uiRegNum);
int32_t WitRegisterCallBack(RegUpdateCb update_func);

int32_t WitReadReg(uint32_t uiReg, uint32_t uiReadNum);
int32_t WitInit(uint32_t uiProtocol, uint8_t ucAddr);

extern int16_t sReg[REGSIZE];

#ifdef __cplusplus
}
#endif

#endif /* __WIT_C_SDK_H */

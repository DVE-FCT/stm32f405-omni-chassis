#ifndef __WIT_GYRO_H
#define __WIT_GYRO_H

#include "usart.h"
#include "wit_c_sdk.h"
#include <string.h>
#include <stdio.h>

/* 数据更新标志 */
#define ACC_UPDATE      0x01
#define GYRO_UPDATE     0x02
#define ANGLE_UPDATE    0x04
#define to_rad          0.0174533f

/* 全局变量 */
extern float fAngle[3];   /* [Roll, Pitch, Yaw] 度 */
extern float fAcc[3];     /* [Ax, Ay, Az] g        */
extern float fGyro[3];    /* [Gx, Gy, Gz] dps      */

void GYR_Init(void);                    /* 初始化陀螺仪 SDK          */
void GYR_Updata(void);                  /* 拉取最新数据 (main循环调) */
void GYR_FeedByte(uint8_t byte);        /* USART2 RX 喂字节给 SDK    */

#endif /* __WIT_GYRO_H */

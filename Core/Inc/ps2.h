#ifndef __PS2_H
#define __PS2_H

#include "main.h"

/* ---- PS2 接收器接线 (CubeMX 已配置) ---- */
/* PB12=DI(MISO) PB13=DO(MOSI) PB8=CS PB9=CLK */
#define PS2_DI_PORT     PS2_DI_GPIO_Port
#define PS2_DI_PIN      PS2_DI_Pin
#define PS2_DO_PORT     PS2_DO_GPIO_Port
#define PS2_DO_PIN      PS2_DO_Pin
#define PS2_CS_PORT     PS2_CS_GPIO_Port
#define PS2_CS_PIN      PS2_CS_Pin
#define PS2_CLK_PORT    PS2_CLK_GPIO_Port
#define PS2_CLK_PIN     PS2_CLK_Pin

/* ---- 按键编号 ---- */
#define PSB_SELECT      1
#define PSB_L3          2
#define PSB_R3          3
#define PSB_START       4
#define PSB_PAD_UP      5
#define PSB_PAD_RIGHT   6
#define PSB_PAD_DOWN    7
#define PSB_PAD_LEFT    8
#define PSB_L2          9
#define PSB_R2          10
#define PSB_L1          11
#define PSB_R1          12
#define PSB_GREEN       13
#define PSB_RED         14
#define PSB_BLUE        15
#define PSB_PINK        16

/* ---- 摇杆通道索引 ---- */
#define PSS_RX          5   /* 右摇杆 X (0~255, 中=128) */
#define PSS_RY          6   /* 右摇杆 Y                 */
#define PSS_LX          7   /* 左摇杆 X                 */
#define PSS_LY          8   /* 左摇杆 Y                 */

/* ---- API ---- */
void PS2_Init(void);
void PS2_ReadData(void);
uint8_t PS2_DataKey(void);            /* 按键值: 0=无, 1~16=按键 */
uint8_t PS2_GetRawByte(uint8_t idx);  /* 读取 Data[idx], 调试用   */
uint8_t PS2_IsConnected(void);        /* ID=0x73/0x41 → 已连接    */
uint8_t PS2_AnalogData(uint8_t ch);   /* 摇杆值: 0~255, 128=中位 */

#endif /* __PS2_H */

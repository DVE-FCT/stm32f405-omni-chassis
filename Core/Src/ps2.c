/**
 * @file    ps2.c
 * @brief   PS2 无线手柄驱动 — 软件 SPI 协议, HAL GPIO 适配
 *
 * PS2 接收器接线:
 *   PB12 (MISO) ← 手柄 DO
 *   PB13 (MOSI) → 手柄 DI
 *   PB8  (CS)   ← 手柄 CS
 *   PB9  (SCLK) ← 手柄 CLK
 *
 * 协议: SPI 模式, 时钟 < 250kHz, 9 字节数据帧
 *   Data[3:4] = 16 位按键掩码 (0=按下)
 *   Data[5]   = 右摇杆 X (0~255, 128=中位)
 *   Data[6]   = 右摇杆 Y
 *   Data[7]   = 左摇杆 X
 *   Data[8]   = 左摇杆 Y
 */

#include "ps2.h"

static uint8_t Data[9];
static uint16_t MASK[] = {
    PSB_SELECT, PSB_L3, PSB_R3, PSB_START,
    PSB_PAD_UP, PSB_PAD_RIGHT, PSB_PAD_DOWN, PSB_PAD_LEFT,
    PSB_L2, PSB_R2, PSB_L1, PSB_R1,
    PSB_GREEN, PSB_RED, PSB_BLUE, PSB_PINK
};

/* ---- HAL GPIO 操作 ---- */
#define DI      HAL_GPIO_ReadPin(PS2_DI_PORT, PS2_DI_PIN)
#define DO_H    HAL_GPIO_WritePin(PS2_DO_PORT, PS2_DO_PIN, GPIO_PIN_SET)
#define DO_L    HAL_GPIO_WritePin(PS2_DO_PORT, PS2_DO_PIN, GPIO_PIN_RESET)
#define CS_H    HAL_GPIO_WritePin(PS2_CS_PORT, PS2_CS_PIN, GPIO_PIN_SET)
#define CS_L    HAL_GPIO_WritePin(PS2_CS_PORT, PS2_CS_PIN, GPIO_PIN_RESET)
#define CLK_H   HAL_GPIO_WritePin(PS2_CLK_PORT, PS2_CLK_PIN, GPIO_PIN_SET)
#define CLK_L   HAL_GPIO_WritePin(PS2_CLK_PORT, PS2_CLK_PIN, GPIO_PIN_RESET)

/* ---- 微秒延时 ---- */
static void delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000);
    while ((DWT->CYCCNT - start) < ticks) { __NOP(); }
}

/* ---- 发送单字节, LSB 优先 ---- */
static void PS2_Cmd(uint8_t CMD)
{
    volatile uint16_t ref = 0x01;
    Data[1] = 0;
    for (ref = 0x01; ref < 0x0100; ref <<= 1) {
        if (ref & CMD) DO_H; else DO_L;
        CLK_H; delay_us(10);
        CLK_L; delay_us(10);
        CLK_H;
        if (DI) Data[1] = ref | Data[1];
    }
    delay_us(16);
}

/**
 * PS2_Init — 初始化接收器 GPIO + 设置手柄模式
 */
void PS2_Init(void)
{
    /* 启用 DWT 周期计数器 (用于精确延时) */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    /* GPIO 由 CubeMX MX_GPIO_Init() 配置, 此处仅设置初始电平 */
    CS_H; CLK_H;

    /* 手柄初始化序列: SHORT_POLL×3 → ENTER_CONFIG → ANALOG_MODE → EXIT_CONFIG */
    for (int i = 0; i < 3; i++) {
        CS_L; delay_us(16);
        PS2_Cmd(0x01); PS2_Cmd(0x42); PS2_Cmd(0x00);
        PS2_Cmd(0x00); PS2_Cmd(0x00);
        CS_H; delay_us(16);
    }
    /* ENTER_CONFIG */
    CS_L; delay_us(16);
    PS2_Cmd(0x01); PS2_Cmd(0x43); PS2_Cmd(0x00);
    PS2_Cmd(0x01); PS2_Cmd(0x00);
    CS_H; delay_us(16);
    /* ANALOG_MODE */
    CS_L;
    PS2_Cmd(0x01); PS2_Cmd(0x44); PS2_Cmd(0x00);
    PS2_Cmd(0x01); PS2_Cmd(0x03); /* analog=on, lock */
    PS2_Cmd(0x00); PS2_Cmd(0x00); PS2_Cmd(0x00); PS2_Cmd(0x00);
    CS_H; delay_us(16);
    /* EXIT_CONFIG */
    CS_L; delay_us(16);
    PS2_Cmd(0x01); PS2_Cmd(0x43); PS2_Cmd(0x00);
    PS2_Cmd(0x00); PS2_Cmd(0x5A); PS2_Cmd(0x5A);
    PS2_Cmd(0x5A); PS2_Cmd(0x5A); PS2_Cmd(0x5A);
    CS_H; delay_us(16);
}

/**
 * PS2_ReadData — 读取 9 字节手柄数据
 */
void PS2_ReadData(void)
{
    CS_L;
    delay_us(16);    /* CS 建立时间 */
    PS2_Cmd(0x01);  /* 开始 */
    PS2_Cmd(0x42);  /* 请求数据 */
    for (int byte = 2; byte < 9; byte++) {
        Data[byte] = 0;  /* 清空旧值, 防止|=残留 0xFF */
        for (uint16_t ref = 0x01; ref < 0x100; ref <<= 1) {
            CLK_H; delay_us(10);
            CLK_L; delay_us(10);
            CLK_H;
            if (DI) Data[byte] = ref | Data[byte];
        }
        delay_us(16);
    }
    CS_H;
    delay_us(50);
}

/**
 * PS2_DataKey — 返回当前按下的按键 (1~16), 0=无
 */
uint8_t PS2_DataKey(void)
{
    /* 清空 Data */
    for (int i = 0; i < 9; i++) Data[i] = 0x00;
    PS2_ReadData();

    uint16_t Handkey = (Data[4] << 8) | Data[3];
    for (uint8_t i = 0; i < 16; i++) {
        if ((Handkey & (1 << (MASK[i] - 1))) == 0)
            return i + 1;
    }
    return 0;
}

uint8_t PS2_AnalogData(uint8_t ch)  { return Data[ch]; }
uint8_t PS2_GetRawByte(uint8_t idx) { return Data[idx]; }
uint8_t PS2_IsConnected(void)       { return (Data[1]==0x73||Data[1]==0x41); }

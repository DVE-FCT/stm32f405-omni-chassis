#ifndef __ENCODER_H
#define __ENCODER_H

#include "main.h"

/* 编码器编号 */
typedef enum {
    ENC_1 = 0,  /* 电机1: TIM3, PA6/PA7 */
    ENC_2 = 1,  /* 电机2: TIM4, PB6/PB7 */
    ENC_3 = 2,  /* 电机3: TIM5, PA0/PA1 */
    ENC_4 = 3,  /* 电机4: TIM8, PC6/PC7 */
} Encoder_ID;

/* 编码器线数 (WHEELTEC 电机编码器规格) */
#define ENCODER_LINES   13      /* 每圈脉冲线数               */
#define ENC_PULSE_MUL   4       /* 4倍频 (TI12双沿触发)       */
#define ENC_PPR         (ENCODER_LINES * ENC_PULSE_MUL) /* 每圈脉冲数 = 52 */

/* 轮子参数 */
#define WHEEL_DIAMETER  0.065f  /* 轮径 65mm = 0.065m          */
#define WHEEL_PERIMETER (3.1415926f * WHEEL_DIAMETER) /* 周长 ≈ 0.204m */
#define GEAR_RATIO      45.0f   /* 减速比 1:45 — 电机轴:输出轴 */

/* 函数声明 */
void Encoder_Init(void);
void Encoder_Update(void);                          /* 每10ms调用, 计算转速   */
int32_t Encoder_GetCount(Encoder_ID id);             /* 读取原始脉冲计数       */
float Encoder_GetRPM(Encoder_ID id);                /* 读取转速 (转/分钟)     */
float Encoder_GetSpeed(Encoder_ID id);              /* 读取线速度 (m/s)       */

#endif /* __ENCODER_H */

#ifndef __MOTOR_H
#define __MOTOR_H

#include "main.h"

/* 电机编号 */
typedef enum {
    MOTOR_1 = 0,
    MOTOR_2 = 1,
    MOTOR_3 = 2,
    MOTOR_4 = 3
} Motor_ID;

/* 电机方向 */
typedef enum {
    MOTOR_STOP = 0,
    MOTOR_FORWARD,
    MOTOR_REVERSE
} Motor_Dir;

/* PWM 参数 */
#define MOTOR1_TIM              htim1
#define MOTOR1_CH               TIM_CHANNEL_1
#define MOTOR1_ARR              8399
#define MOTOR2_TIM              htim1
#define MOTOR2_CH               TIM_CHANNEL_4
#define MOTOR2_ARR              8399
#define MOTOR3_TIM              htim12
#define MOTOR3_CH               TIM_CHANNEL_1
#define MOTOR3_ARR              4199
#define MOTOR4_TIM              htim12
#define MOTOR4_CH               TIM_CHANNEL_2
#define MOTOR4_ARR              4199

/* 函数声明 */
void Motor_Init(void);
void Motor_SetSpeed(Motor_ID id, int16_t speed);
void Motor_Stop(Motor_ID id);
void Motor_StopAll(void);

#endif /* __MOTOR_H */

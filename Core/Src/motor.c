/**
 * @file    motor.c
 * @brief   4路直流电机驱动 — 基于 2×TB6612FNG 模块
 *
 * 硬件连接:
 *   电机1: TIM1_CH1 (PA8  PWM) + PC0(AIN1) + PC1(AIN2)
 *   电机2: TIM1_CH4 (PA11 PWM) + PC2(AIN1) + PC3(AIN2)
 *   电机3: TIM12_CH1(PB14 PWM) + PC4(AIN1) + PC5(AIN2)
 *   电机4: TIM12_CH2(PB15 PWM) + PB0(AIN1) + PB1(AIN2)
 *
 * TB6612FNG 真值表:
 *   AIN1=1, AIN2=0, PWM=X  → 正转 (电流 A→B)
 *   AIN1=0, AIN2=1, PWM=X  → 反转 (电流 B→A)
 *   AIN1=1, AIN2=1, PWM=X  → 短路刹车 (电机两端接地)
 *   AIN1=0, AIN2=0, PWM=X  → 滑行 (高阻态, 无制动力)
 *
 * PWM 参数:
 *   电机1/2 使用 TIM1 (APB2=168MHz), PSC=0, ARR=8399 → 20kHz
 *   电机3/4 使用 TIM12(APB1=84MHz),  PSC=0, ARR=4199 → 20kHz
 */

#include "motor.h"
#include "tim.h"
#include "gpio.h"

/* ---------------------------------------------------------------
 * 每个电机的硬件配置结构体
 * --------------------------------------------------------------- */
typedef struct {
    TIM_HandleTypeDef *htim;    /* 定时器句柄          */
    uint32_t channel;           /* PWM 通道 (CH1~CH4)  */
    uint16_t arr;               /* 自动重载值 (0~ARR)  */
    GPIO_TypeDef *dir_port;     /* 方向控制端口         */
    uint16_t pin1;              /* AIN1 引脚编号       */
    uint16_t pin2;              /* AIN2 引脚编号       */
} Motor_Cfg;

/**
 * 电机配置表
 * 索引按 Motor_ID 枚举 (MOTOR_1=0, MOTOR_2=1, MOTOR_3=2, MOTOR_4=3)
 */
static const Motor_Cfg motor_cfg[4] = {
    [MOTOR_1] = { &MOTOR1_TIM, MOTOR1_CH, MOTOR1_ARR, GPIOC, M1_DIR1_Pin, M1_DIR2_Pin },
    [MOTOR_2] = { &MOTOR2_TIM, MOTOR2_CH, MOTOR2_ARR, GPIOC, M2_DIR1_Pin, M2_DIR2_Pin },
    [MOTOR_3] = { &MOTOR3_TIM, MOTOR3_CH, MOTOR3_ARR, GPIOC, M3_DIR1_Pin, M3_DIR2_Pin },
    [MOTOR_4] = { &MOTOR4_TIM, MOTOR4_CH, MOTOR4_ARR, GPIOB, M4_DIR1_Pin, M4_DIR2_Pin },
};

/* ---------------------------------------------------------------
 * Motor_Init
 *
 * 初始化所有电机:
 *   1. 启动 4 路 PWM 输出 (CubeMX 只 Init 不 Start)
 *   2. 全部设为滑行/停止状态
 * --------------------------------------------------------------- */
void Motor_Init(void)
{
    /* 启动 PWM: HAL_TIM_PWM_Start 使能定时器通道输出 */
    HAL_TIM_PWM_Start(&MOTOR1_TIM, MOTOR1_CH);
    HAL_TIM_PWM_Start(&MOTOR1_TIM, MOTOR2_CH);
    HAL_TIM_PWM_Start(&MOTOR3_TIM, MOTOR3_CH);
    HAL_TIM_PWM_Start(&MOTOR4_TIM, MOTOR4_CH);

    /* 初始化全部停止 */
    for (int i = 0; i < 4; i++) {
        Motor_Stop((Motor_ID)i);
    }
}

/* ---------------------------------------------------------------
 * Motor_SetSpeed
 *
 * 设置指定电机的转速
 * @param id    电机编号 (MOTOR_1 ~ MOTOR_4)
 * @param speed 转速 (-100 ~ 100), 负数为反转, 0 为刹车
 *
 * 内部逻辑:
 *   speed >  0 → AIN1=1, AIN2=0, PWM = speed% * ARR  (正转)
 *   speed <  0 → AIN1=0, AIN2=1, PWM = |speed|% * ARR (反转)
 *   speed == 0 → AIN1=1, AIN2=1, PWM = 0            (刹车, 快速停转)
 * --------------------------------------------------------------- */
void Motor_SetSpeed(Motor_ID id, int16_t speed)
{
    /* 限幅: -100 ~ +100 */
    if (speed > 100)  speed = 100;
    if (speed < -100) speed = -100;

    const Motor_Cfg *m = &motor_cfg[id];

    if (speed > 0) {
        /* ===== 正转: AIN1=1, AIN2=0 ===== */
        HAL_GPIO_WritePin(m->dir_port, m->pin1, GPIO_PIN_SET);
        HAL_GPIO_WritePin(m->dir_port, m->pin2, GPIO_PIN_RESET);
        /* __HAL_TIM_SET_COMPARE 直接写 CCR 寄存器, 无需 HAL 开销 */
        uint16_t pulse = (uint16_t)((uint32_t)m->arr * speed / 100);
        __HAL_TIM_SET_COMPARE(m->htim, m->channel, pulse);

    } else if (speed < 0) {
        /* ===== 反转: AIN1=0, AIN2=1 ===== */
        HAL_GPIO_WritePin(m->dir_port, m->pin1, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(m->dir_port, m->pin2, GPIO_PIN_SET);
        uint16_t pulse = (uint16_t)((uint32_t)m->arr * (-speed) / 100);
        __HAL_TIM_SET_COMPARE(m->htim, m->channel, pulse);

    } else {
        /* ===== 刹车: AIN1=1, AIN2=1, PWM=0 =====
         * 电机两端短接, 旋转时产生反向制动电流, 快速停转
         * 注意: 刹车功耗较大, 长时间使用需注意 TB6612 发热 */
        HAL_GPIO_WritePin(m->dir_port, m->pin1, GPIO_PIN_SET);
        HAL_GPIO_WritePin(m->dir_port, m->pin2, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(m->htim, m->channel, 0);
    }
}

/* ---------------------------------------------------------------
 * Motor_Stop
 *
 * 停止指定电机 (滑行模式)
 *   AIN1=0, AIN2=0, PWM=0
 *   TB6612 输出高阻态, 电机可自由转动, 无制动效果
 *
 * 与 speed=0 的刹车模式不同: Stop 是滑行, speed=0 是刹车
 * --------------------------------------------------------------- */
void Motor_Stop(Motor_ID id)
{
    const Motor_Cfg *m = &motor_cfg[id];
    HAL_GPIO_WritePin(m->dir_port, m->pin1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(m->dir_port, m->pin2, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(m->htim, m->channel, 0);
}

/* ---------------------------------------------------------------
 * Motor_StopAll
 *
 * 停止所有 4 路电机 (滑行)
 * --------------------------------------------------------------- */
void Motor_StopAll(void)
{
    for (int i = 0; i < 4; i++) {
        Motor_Stop((Motor_ID)i);
    }
}

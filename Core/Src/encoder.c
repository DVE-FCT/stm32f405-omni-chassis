/**
 * @file    encoder.c
 * @brief   4路编码器读取 — TIM3/4/5/8 编码器模式
 *
 * 硬件连接:
 *   编码器1: TIM3_CH1(PA6) + CH2(PA7)  → 电机1
 *   编码器2: TIM4_CH1(PB6) + CH2(PB7)  → 电机2
 *   编码器3: TIM5_CH1(PA0) + CH2(PA1)  → 电机3
 *   编码器4: TIM8_CH1(PC6) + CH2(PC7)  → 电机4
 *
 * 工作原理:
 *   定时器配置为 Encoder Mode TI12, A/B两相双沿均触发
 *   每圈脉冲数 = 编码器线数 × 4倍频 = ENCODER_LINES × ENC_PULSE_MUL
 *   方向识别: 硬件自动, A相超前B相→加计数, B相超前A相→减计数
 *
 *   转速计算 (每10ms调用 Encoder_Update):
 *     RPM = Δ脉冲 / PPR / Δt(分钟)
 *         = (cnt - last_cnt) / 52 / (0.01/60)
 *         = (cnt - last_cnt) * 115.38...
 */

#include "encoder.h"
#include "tim.h"

/* ---------------------------------------------------------------
 * 每个编码器的配置
 * --------------------------------------------------------------- */
typedef struct {
    TIM_HandleTypeDef *htim;    /* 定时器句柄 (TIM3/4/5/8) */
} Enc_Cfg;

static const Enc_Cfg enc_cfg[4] = {
    [ENC_1] = { &htim3 },
    [ENC_2] = { &htim4 },
    [ENC_3] = { &htim5 },
    [ENC_4] = { &htim8 },
};

/* 上一周期: 编码器计数值 + 时间戳 */
static int32_t  enc_last_cnt[4] = {0};
static uint32_t enc_last_tick = 0;          /* HAL_GetTick() 时间戳 */
/* 当前转速: 0.0 ~ N RPM, 正=正转, 负=反转 */
static float    enc_rpm[4] = {0};

/* ---------------------------------------------------------------
 * Encoder_Init
 *
 * 初始化编码器, 启动 4 路编码器定时器
 * CubeMX 的 MX_XXX_Init() 只配置不启动, 这里启动计数
 * --------------------------------------------------------------- */
void Encoder_Init(void)
{
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);
}

/* ---------------------------------------------------------------
 * Encoder_Update
 *
 * 更新转速计算 (每 10ms 在 main 循环中调用)
 *
 * 计算公式:
 *   RPM = Δ脉冲 / PPR / Δt
 *   PPR = 13线 × 4倍频 = 52 脉冲/圈
 *   Δt  = 0.01秒 = 1/6000 分钟
 *   RPM = (cnt - last) / 52 * 6000
 *       ≈ (cnt - last) * 115.3846
 * --------------------------------------------------------------- */
void Encoder_Update(void)
{
    /* 计算实际时间间隔 (秒) */
    uint32_t now = HAL_GetTick();
    float dt = (float)(now - enc_last_tick) / 1000.0f;  /* ms → s */
    if (dt <= 0.0f) dt = 0.01f;                         /* 防止除零 */
    enc_last_tick = now;

    for (int i = 0; i < 4; i++) {
        /* 读取当前计数 (16位定时器, 0~65535) */
        uint16_t cnt = (uint16_t)__HAL_TIM_GET_COUNTER(enc_cfg[i].htim);

        /* 16位回绕安全差分
         * (uint16_t)new - (uint16_t)old → int16_t
         * 自动处理溢出: 500 - 65000 → int16_t(1536) = 1536 ✓ */
        int32_t delta = (int16_t)(cnt - (uint16_t)enc_last_cnt[i]);
        enc_last_cnt[i] = cnt;

        /* 瞬时 RPM = delta / PPR / (dt/60) = delta * 60 / (PPR * dt) */
        enc_rpm[i] = (float)delta * 60.0f / (ENC_PPR * dt);
    }
}

/* ---------------------------------------------------------------
 * Encoder_GetCount
 *
 * 读取编码器原始脉冲计数值
 * 可用于绝对位置估算或里程计积分
 * --------------------------------------------------------------- */
int32_t Encoder_GetCount(Encoder_ID id)
{
    return enc_last_cnt[id];
}

/* ---------------------------------------------------------------
 * Encoder_GetRPM
 *
 * 获取最近一次更新的转速 (转/分钟)
 * 正值为正转, 负值为反转
 * --------------------------------------------------------------- */
float Encoder_GetRPM(Encoder_ID id)
{
    return enc_rpm[id];
}

/* ---------------------------------------------------------------
 * Encoder_GetSpeed
 *
 * 获取线速度 (m/s)
 * v = RPM × 周长 / 60
 *   = RPM × (π × 65mm) / 60
 *   = RPM × 0.2042 / 60
 *   = RPM × 0.003403
 * 正值为前进, 负值为后退
 * --------------------------------------------------------------- */
float Encoder_GetSpeed(Encoder_ID id)
{
    return enc_rpm[id] * WHEEL_PERIMETER / (60.0f * GEAR_RATIO);
}

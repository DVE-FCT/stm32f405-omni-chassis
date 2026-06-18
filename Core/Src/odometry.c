/**
 * @file    odometry.c
 * @brief   麦轮底盘里程计 + IMU融合 + 打滑检测
 */

#include "odometry.h"
#include "encoder.h"
#include <math.h>

static float pos_x = 0.0f, pos_y = 0.0f;
static float theta_enc = 0.0f, theta_fused = 0.0f;

/* IMU 数据 */
static float gyro_z = 0.0f, imu_yaw = 0.0f;
static float acc_ax = 0.0f, acc_ay = 0.0f;

/* 编码器速度历史 */
static float vx_last = 0.0f, vy_last = 0.0f;
static float w_enc_last = 0.0f;

/* 有效标志 */
static uint8_t gyro_valid = 0, imu_valid = 0;

/* ---- 打滑检测状态 ---- */
static uint8_t  slip_flag = 0;
static uint8_t  slip_enter_cnt = 0, slip_exit_cnt = 0;
static float    a_enc_f = 0.0f, a_imu_f = 0.0f;  /* 低通滤波 */

#define SLIP_ENTER_CNT  3     /* 连续 N 次 → 进入打滑 */
#define SLIP_EXIT_CNT   6     /* 连续 N 次 → 退出打滑 */
#define A_ENC_THR       0.8f  /* 编码器加速度阈值 (m/s²) */
#define A_IMU_LOW       0.25f /* IMU 低加速度阈值 (m/s²) */
#define A_ERR_EXIT      0.5f  /* 退出: 编码器与IMU加速度差异 */
#define V_STOP          0.03f /* 退出: 车体速度接近0 (m/s) */
#define W_SLIP_THR      0.8f  /* 旋转打滑: w_enc 阈值 (rad/s) */
#define GZ_SLIP_LOW     0.2f  /* 旋转打滑: gyro_z 低阈值 */

static float wrap_pi(float a) {
    while (a >  M_PI) a -= 2.0f * M_PI;
    while (a < -M_PI) a += 2.0f * M_PI;
    return a;
}
static float angle_error(float t, float c) { return wrap_pi(t - c); }

void Odometry_Init(void)
{
    pos_x = pos_y = 0.0f;
    theta_enc = theta_fused = 0.0f;
    gyro_z = imu_yaw = 0.0f;
    acc_ax = acc_ay = 0.0f;
    vx_last = vy_last = w_enc_last = 0.0f;
    gyro_valid = imu_valid = 0;
    slip_flag = slip_enter_cnt = slip_exit_cnt = 0;
    a_enc_f = a_imu_f = 0.0f;
}

void Odometry_Update(float dt)
{
    if (dt <= 0.0f || dt > 0.1f) dt = 0.02f;

    float v1 = Encoder_GetSpeed(ENC_1);
    float v2 = Encoder_GetSpeed(ENC_2);
    float v3 = Encoder_GetSpeed(ENC_3);
    float v4 = Encoder_GetSpeed(ENC_4);

    /* 正解: v1=-F-S-R v2=+F-S+R v3=+F+S-R v4=-F+S+R */
    float vx = (v2 + v3) / 2.0f;
    float vy = (v3 - v2 + v4 - v1) / 4.0f;
    float R   = (v2 - v1 + v4 - v3) / 4.0f;  /* R = ω×L */
    float w_enc = R / ODOM_L;
    w_enc_last = w_enc;

    /* 编码器航向 (死区) */
    if (fabsf(w_enc) > 0.1f)
        theta_enc = wrap_pi(theta_enc + w_enc * dt);

    /* 互补滤波: 陀螺仪预测 + IMU绝对航向修正 */
    if (gyro_valid && imu_valid) {
        float tp = wrap_pi(theta_fused + gyro_z * dt);
        theta_fused = wrap_pi(tp + (1.0f - FUSION_ALPHA) * angle_error(imu_yaw, tp));
    } else if (imu_valid) {
        theta_fused = imu_yaw;
    } else {
        theta_fused = theta_enc;
    }

    /* ===== 打滑检测 ===== */
    float enc_ax = (dt > 0.001f) ? (vx - vx_last) / dt : 0;
    float enc_ay = (dt > 0.001f) ? (vy - vy_last) / dt : 0;
    vx_last = vx; vy_last = vy;

    float a_enc = sqrtf(enc_ax*enc_ax + enc_ay*enc_ay);
    float a_imu = sqrtf(acc_ax*acc_ax + acc_ay*acc_ay);

    /* 低通滤波 (截止 ~5Hz) */
    a_enc_f = 0.8f * a_enc_f + 0.2f * a_enc;
    a_imu_f = 0.8f * a_imu_f + 0.2f * a_imu;

    float a_err = fabsf(a_enc_f - a_imu_f);
    float v_body = sqrtf(vx*vx + vy*vy);
    float w_err = fabsf(w_enc - gyro_z);

    /* 进入打滑: 编码器有加速度但IMU没有, 或旋转不一致 */
    uint8_t xy_slip = (a_enc_f > A_ENC_THR && a_imu_f < A_IMU_LOW);
    uint8_t rot_slip = (fabsf(w_enc) > W_SLIP_THR && fabsf(gyro_z) < GZ_SLIP_LOW);

    if (!slip_flag) {
        if (xy_slip || rot_slip) {
            if (++slip_enter_cnt >= SLIP_ENTER_CNT) {
                slip_flag = 1;
                slip_enter_cnt = 0;
                slip_exit_cnt = 0;
            }
        } else { slip_enter_cnt = 0; }
    } else {
        uint8_t stopped = v_body < V_STOP;
        uint8_t agree  = a_err < A_ERR_EXIT && a_enc_f > 0.2f && a_imu_f > 0.2f;
        if (stopped || agree) {
            if (++slip_exit_cnt >= SLIP_EXIT_CNT) {
                slip_flag = 0;
                slip_exit_cnt = 0;
                vx_last = vx; vy_last = vy;
            }
        } else { slip_exit_cnt = 0; }
    }

    /* 世界坐标积分 (打滑时冻结位置) */
    if (!slip_flag) {
        float c = cosf(theta_fused);
        float s = sinf(theta_fused);
        pos_x += (vx * c - vy * s) * dt;
        pos_y += (vx * s + vy * c) * dt;
    }
}

void Odometry_SetGyroZ(float gz)  { gyro_z = gz;  gyro_valid = 1; }
void Odometry_SetImuYaw(float y)  { imu_yaw = y;  imu_valid = 1; }
void Odometry_SetAccel(float ax, float ay) { acc_ax = ax; acc_ay = ay; }

uint8_t Odometry_IsSlipping(void) { return slip_flag; }
float Odometry_GetGyroZ(void)  { return gyro_z; }
float Odometry_GetWenc(void)   { return w_enc_last; }
float Odometry_GetX(void)      { return pos_x; }
float Odometry_GetY(void)      { return pos_y; }
float Odometry_GetTheta(void)  { return theta_fused; }

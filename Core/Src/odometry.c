/**
 * @file    odometry.c
 * @brief   麦轮底盘里程计 — 编码器轮速 → 位姿 (x, y, θ)
 *
 * 使用 4 路编码器线速度 (Encoder_GetSpeed), 经麦轮正运动学
 * 解算为底盘速度 (vx, vy, w), 再积分到世界坐标 (x, y, θ)。
 *
 * 方向修正: 与 Motor_SetSpeed 的方向系数一致
 *   M1=-1  M2=+1  M3=+1  M4=-1
 */

#include "odometry.h"
#include "encoder.h"
#include <math.h>

/* 当前位姿 */
static float pos_x = 0.0f, pos_y = 0.0f, pos_theta = 0.0f;

/**
 * Odometry_Init — 复位里程计到原点
 */
void Odometry_Init(void)
{
    pos_x = 0.0f;
    pos_y = 0.0f;
    pos_theta = 0.0f;
}

/**
 * Odometry_Update — 读取编码器并更新位姿
 *
 * @param dt  距离上次调用的时间间隔 (秒)
 *
 * 正运动学 (轮速 → 底盘速度):
 *   vx = (v1 + v2 + v3 + v4) / 4
 *   vy = (-v1 + v2 - v3 + v4) / 4
 *   w  = (-v1 + v2 + v3 - v4) / (4 * L)
 *
 * 世界坐标积分:
 *   x += (vx*cos(θ) - vy*sin(θ)) * dt
 *   y += (vx*sin(θ) + vy*cos(θ)) * dt
 *   θ += w * dt
 */
void Odometry_Update(float dt)
{
    if (dt <= 0.0f) dt = 0.01f;

    /* 编码器线速度 (m/s), 方向已修正 */
    float v1 = -Encoder_GetSpeed(ENC_1);  /* M1 左前, 方向系数 -1 */
    float v2 =  Encoder_GetSpeed(ENC_2);  /* M2 左后, 方向系数 +1 */
    float v3 =  Encoder_GetSpeed(ENC_3);  /* M3 右前, 方向系数 +1 */
    float v4 = -Encoder_GetSpeed(ENC_4);  /* M4 右后, 方向系数 -1 */

    /* 麦轮正运动学 → 底盘速度 */
    float vx = (v1 + v2 + v3 + v4) / 4.0f;
    float vy = (-v1 + v2 - v3 + v4) / 4.0f;
    float w  = (-v1 + v2 + v3 - v4) / (4.0f * ODOM_L);

    /* 世界坐标积分 */
    float c = cosf(pos_theta);
    float s = sinf(pos_theta);
    pos_x += (vx * c - vy * s) * dt;
    pos_y += (vx * s + vy * c) * dt;
    pos_theta += w * dt;

    /* 角度归一化到 [-π, π) */
    while (pos_theta >  M_PI) pos_theta -= 2.0f * M_PI;
    while (pos_theta < -M_PI) pos_theta += 2.0f * M_PI;
}

/**
 * Odometry_GetX — X 坐标 (m)
 */
float Odometry_GetX(void) { return pos_x; }

/**
 * Odometry_GetY — Y 坐标 (m)
 */
float Odometry_GetY(void) { return pos_y; }

/**
 * Odometry_GetTheta — 偏航角 (rad)
 */
float Odometry_GetTheta(void) { return pos_theta; }

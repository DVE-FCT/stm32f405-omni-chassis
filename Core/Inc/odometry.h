#ifndef __ODOMETRY_H
#define __ODOMETRY_H

#include "main.h"

/* 底盘尺寸 (实测: 左右20cm, 前后20cm) */
#define WHEEL_BASE_LX   0.10f   /* 左右轮距半长 (m) = 20cm/2 */
#define WHEEL_BASE_LY   0.10f   /* 前后轮距半长 (m) = 20cm/2 */
#define ODOM_L          (WHEEL_BASE_LX + WHEEL_BASE_LY) /* = 0.20m */

/* 互补滤波系数 (0~1, 越大越信任陀螺仪)
 * θ_fused = α×θ_fused + (1-α)×θ_imu_yaw  (IMU绝对航向慢修正)
 * 陀螺仪: 角速度直接测量, 无打滑误差
 * IMU航向: 绝对参考, 防止编码器微量噪声累积漂移 */
#define FUSION_ALPHA    0.98f

/* API */
void Odometry_Init(void);
void Odometry_Update(float dt);          /* dt=秒, main 循环中调用           */
void Odometry_SetGyroZ(float gyro_z_rad_s); /* 注入角速度, 互补滤波预测步          */
void Odometry_SetImuYaw(float yaw_rad);     /* 注入 IMU 绝对航向, 互补滤波修正步     */
void Odometry_SetAccel(float ax, float ay); /* 注入加速度 (m/s²), 用于打滑检测       */
uint8_t Odometry_IsSlipping(void);          /* 返回 1=打滑, 0=正常                  */
float Odometry_GetGyroZ(void);              /* 返回注入的 IMU 角速度 (诊断用)       */
float Odometry_GetWenc(void);               /* 返回编码器解算的 w (诊断用)          */
float Odometry_GetX(void);                  /* X 坐标 (m)                          */
float Odometry_GetY(void);                  /* Y 坐标 (m)                          */
float Odometry_GetTheta(void);              /* 偏航角 (rad), 融合后                */

#endif /* __ODOMETRY_H */

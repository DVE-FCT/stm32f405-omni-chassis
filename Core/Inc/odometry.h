#ifndef __ODOMETRY_H
#define __ODOMETRY_H

#include "main.h"

/* 底盘尺寸 (实测: 左右20cm, 前后20cm) */
#define WHEEL_BASE_LX   0.10f   /* 左右轮距半长 (m) = 20cm/2 */
#define WHEEL_BASE_LY   0.10f   /* 前后轮距半长 (m) = 20cm/2 */
#define ODOM_L          (WHEEL_BASE_LX + WHEEL_BASE_LY) /* = 0.20m */

/* API */
void Odometry_Init(void);
void Odometry_Update(float dt);          /* dt=秒, main 循环中调用 */
float Odometry_GetX(void);               /* X 坐标 (m)            */
float Odometry_GetY(void);               /* Y 坐标 (m)            */
float Odometry_GetTheta(void);           /* 偏航角 (rad)          */

#endif /* __ODOMETRY_H */

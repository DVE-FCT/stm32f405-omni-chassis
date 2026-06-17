/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.h
  * @brief   This file contains all the function prototypes for
  *          the adc.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ADC_H__
#define __ADC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern ADC_HandleTypeDef hadc1;

/* USER CODE BEGIN Private defines */
/* 电池电压监测参数 (D24A 模块 11:1 电阻分压) */
#define ADC_VOLTAGE_DIVIDER    11      /* 分压比: V_bat / V_pin   */
#define ADC_VREF               3.3f    /* ADC 参考电压 (V)        */
#define ADC_RESOLUTION         4096    /* 12位 ADC 满量程 (0~4095) */

/* 电池保护阈值 (3S 锂电池) */
#define BATTERY_LOW_THRESHOLD  9.0f    /* 低压警告 (单节 < 3.0V) */
#define BATTERY_CRIT_THRESHOLD 8.4f    /* 严重低压 (单节 < 2.8V)  */
/* USER CODE END Private defines */

void MX_ADC1_Init(void);

/* USER CODE BEGIN Prototypes */
float ADC_ReadBatteryVoltage(void);     /* 读取电池电压 (V)         */
uint8_t ADC_CheckBatteryLow(void);      /* 返回 0=正常,1=低压,2=严重 */
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __ADC_H__ */


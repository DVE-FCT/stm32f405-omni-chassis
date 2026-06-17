/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "motor.h"
#include "encoder.h"
#include "wit_gyro/wit_gyro.h"
#include "ps2.h"
#include "odometry.h"
#include <stdlib.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern UART_HandleTypeDef huart1;

/* printf 重定向 USART1 */
int _write(int fd, char *ptr, int len)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)ptr, len, 100);
    return len;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_TIM12_Init();
  MX_USART3_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  Motor_Init();
  Encoder_Init();
  GYR_Init();
  PS2_Init();
  Odometry_Init();
  HAL_Delay(500); /* 等待陀螺仪启动 */

  printf("\r\n====================================\r\n");
  printf("STM32F405 Chassis Control V1.0\r\n");
  printf("SYSCLK: 168 MHz, HSE: 8 MHz\r\n");
  printf("PS2 Ready | 4x Motor + Encoder + Gyro\r\n");
  int startup_vbat = (int)(ADC_ReadBatteryVoltage() * 10);
  printf("Battery: %d.%dV\r\n", startup_vbat / 10, startup_vbat % 10);
  printf("====================================\r\n");
  printf("[PS2遥控] 左杆↑↓=前进后退 | 左杆←→=旋转 | 右杆←→=平移\r\n\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* ---- PS2 手柄读取 ---- */
    PS2_ReadData();
    uint8_t lx_raw = PS2_AnalogData(PSS_LX);
    uint8_t ly_raw = PS2_AnalogData(PSS_LY);
    uint8_t rx_raw = PS2_AnalogData(PSS_RX);
    int forward = 0, rotate = 0, strafe = 0;

    if (!PS2_IsConnected()) {
        Motor_StopAll();
        static uint32_t dbg_tick = 0;
        if (HAL_GetTick() - dbg_tick > 2000) {
            printf("[PS2] NO_SIG ID=%02X\r\n", PS2_GetRawByte(1));
            dbg_tick = HAL_GetTick();
        }
    } else {
        /* 左杆↑↓=前进 左杆←→=旋转 右杆←→=平移 */
        forward = 128 - (int)ly_raw;
        rotate  = (int)lx_raw - 128;
        strafe  = 128 - (int)rx_raw;
        if (abs(forward) < 15) forward = 0;
        if (abs(rotate)  < 15) rotate  = 0;
        if (abs(strafe)  < 15) strafe  = 0;

        /* 麦轮: M1左前 M2左后 M3右前 M4右后 */
        int m1 = forward + strafe + rotate;
        int m2 = forward - strafe + rotate;
        int m3 = forward - strafe - rotate;
        int m4 = forward + strafe - rotate;

        int max_abs = abs(m1);
        if (abs(m2) > max_abs) max_abs = abs(m2);
        if (abs(m3) > max_abs) max_abs = abs(m3);
        if (abs(m4) > max_abs) max_abs = abs(m4);
        if (max_abs < 128) max_abs = 128;
        int t[4];
        t[0] = m1 * 100 / max_abs;
        t[1] = m2 * 100 / max_abs;
        t[2] = m3 * 100 / max_abs;
        t[3] = m4 * 100 / max_abs;

        static int cur[4] = {0};
        int ramp = 10;
        for (int i = 0; i < 4; i++) {
            if (cur[i] < t[i]) cur[i] = (cur[i]+ramp < t[i]) ? cur[i]+ramp : t[i];
            else               cur[i] = (cur[i]-ramp > t[i]) ? cur[i]-ramp : t[i];
        }

        /* 方向系数: M1=-1 M2=+1 M3=+1 M4=-1 (实测) */
        Motor_SetSpeed(MOTOR_1, -cur[0]);
        Motor_SetSpeed(MOTOR_2,  cur[1]);
        Motor_SetSpeed(MOTOR_3,  cur[2]);
        Motor_SetSpeed(MOTOR_4, -cur[3]);
    }

    /* ---- 里程计: 每循环更新一次 (50Hz) ---- */
    static uint32_t last_odom_tick = 0;
    uint32_t now_odom = HAL_GetTick();
    float odom_dt = (float)(now_odom - last_odom_tick) / 1000.0f;
    if (odom_dt > 0.5f) odom_dt = 0.02f;  /* 首次/溢出保护 */
    Odometry_Update(odom_dt);
    last_odom_tick = now_odom;

    /* ---- 遥测 (5Hz) ---- */
    static uint32_t last_print = 0;
    if (HAL_GetTick() - last_print >= 200) {
        Encoder_Update(); GYR_Updata();
        int vb = (int)(ADC_ReadBatteryVoltage() * 10);
        int ox = (int)(Odometry_GetX() * 100);   /* cm */
        int oy = (int)(Odometry_GetY() * 100);
        int ot = (int)(Odometry_GetTheta() * 57.3f); /* rad→deg */
        printf(" Vbat:%d.%dV | Odo:(%d,%d) %ddeg | Yaw:%d | F:%d R:%d S:%d\r\n",
               vb/10, vb%10, ox, oy, ot, (int)fAngle[2],
               forward, rotate, strafe);
        last_print = HAL_GetTick();
    }
    HAL_Delay(20);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

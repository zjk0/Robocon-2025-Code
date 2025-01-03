/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#include "can.h"
#include "fatfs.h"
#include "sdio.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "DeepJ60_Motor.h"
#include "leg_control.h"

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

uint8_t MoveEnable = 0;

float J60Motor_StandUpData_CAN1[4] = {0.647773742, -1.65866851, -0.340309143, 1.9198226922}; // lf_out, lf_in, rf_out, rf_in
float J60Motor_StandUpData_CAN2[4] = {2.20684433, -0.66669464111, -1.8807601922, 0.51845550533};     // rb_out, rb_in, lb_out, lb_in
float J60Motor_Data_CAN1[4] = {0, 0, 0, 0};
float J60Motor_Data_CAN2[4] = {0, 0, 0, 0};
float total_time = 1000;
float angle[4][2];
float pos[4][2];
float t = 0;
int flag1 = 0, flag2 = 0, flag3 = 0, flag4 = 0;
float speed = 0.01;

float CubicSpline(float init_position, float goal_position, float init_velocity, float goal_velocity, float now_time, float total_time) {
  float a, b, c, d;
  a = (goal_velocity * total_time + init_velocity * total_time - 2 * goal_position + 2 * init_position) / pow(total_time, 3);
  b = (3 * goal_position - 3 * init_position - 2 * init_velocity * total_time - goal_velocity * total_time) / pow(total_time, 2);
  c = init_velocity;
  d = init_position;

  float now_position = a * pow(now_time, 3) + b * pow(now_time, 2) + c * now_time + d;

  return now_position;
}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_SDIO_SD_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */

  HAL_Delay(4000);

  EnableJ60Motor(&J60Motor_CAN1[0], 1, 1); // lf out
  EnableJ60Motor(&J60Motor_CAN1[1], 2, 1); // lf in
  EnableJ60Motor(&J60Motor_CAN2[0], 1, 2); // rb out
  EnableJ60Motor(&J60Motor_CAN2[1], 2, 2); // rb in

  HAL_Delay(1);
  RunJ60Motor(&J60Motor_CAN1[0], J60Motor_StandUpData_CAN1[0], 0, 0, 20, 5, PositionMode);
  HAL_Delay(1);
  RunJ60Motor(&J60Motor_CAN1[1], J60Motor_StandUpData_CAN1[1], 0, 0, 20, 5, PositionMode);
  HAL_Delay(1);
  RunJ60Motor(&J60Motor_CAN2[0], J60Motor_StandUpData_CAN2[0], 0, 0, 20, 5, PositionMode);
  HAL_Delay(1);
  RunJ60Motor(&J60Motor_CAN2[1], J60Motor_StandUpData_CAN2[1], 0, 0, 20, 5, PositionMode);

  // EnableJ60Motor(&J60Motor_CAN1[2], 3, 1);  // rf out
  // EnableJ60Motor(&J60Motor_CAN1[3], 4, 1);  // rf in
  // EnableJ60Motor(&J60Motor_CAN2[2], 3, 2);  // lb out
  // EnableJ60Motor(&J60Motor_CAN2[3], 4, 2);  // lb in

  // HAL_Delay(1);
  // RunJ60Motor(&J60Motor_CAN1[2], J60Motor_StandUpData_CAN1[2], 0, 0, 20, 5, PositionMode);
  // HAL_Delay(1);
  // RunJ60Motor(&J60Motor_CAN1[3], J60Motor_StandUpData_CAN1[3], 0, 0, 20, 5, PositionMode);
  // HAL_Delay(1);
  // RunJ60Motor(&J60Motor_CAN2[2], J60Motor_StandUpData_CAN2[2], 0, 0, 20, 5, PositionMode);
  // HAL_Delay(1);
  // RunJ60Motor(&J60Motor_CAN2[3], J60Motor_StandUpData_CAN2[3], 0, 0, 20, 5, PositionMode);

  HAL_Delay(2000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // HAL_Delay(20);
    // RunJ60Motor(&J60Motor_CAN1[0], 0, 0, 0, 0, 0, ZeroTorqueMode);
    // HAL_Delay(20);
    // RunJ60Motor(&J60Motor_CAN1[1], 0, 0, 0, 0, 0, ZeroTorqueMode);
    // HAL_Delay(20);
    // RunJ60Motor(&J60Motor_CAN2[0], 0, 0, 0, 0, 0, ZeroTorqueMode);
    // HAL_Delay(20);
    // RunJ60Motor(&J60Motor_CAN2[1], 0, 0, 0, 0, 0, ZeroTorqueMode);

    // HAL_Delay(20);
    // RunJ60Motor(&J60Motor_CAN1[2], 0, 0, 0, 0, 0, ZeroTorqueMode);
    // HAL_Delay(20);
    // RunJ60Motor(&J60Motor_CAN1[3], 0, 0, 0, 0, 0, ZeroTorqueMode);
    // HAL_Delay(20);
    // RunJ60Motor(&J60Motor_CAN2[2], 0, 0, 0, 0, 0, ZeroTorqueMode);
    // HAL_Delay(20);
    // RunJ60Motor(&J60Motor_CAN2[3], 0, 0, 0, 0, 0, ZeroTorqueMode);

    if (MoveEnable == 1) {
      Leg_cyloid(&t, &angle[0][0], &angle[0][1], 0);
      RunJ60Motor(&J60Motor_CAN1[0], J60Motor_StandUpData_CAN1[0] - angle[0][1], 0, 0, 100, 5, PositionMode);
      HAL_Delay(1);
      RunJ60Motor(&J60Motor_CAN1[1], J60Motor_StandUpData_CAN1[1] + angle[0][0], 0, 0, 100, 5, PositionMode);
      HAL_Delay(1);

      Leg_cyloid(&t, &angle[2][0], &angle[2][1], 0);
      RunJ60Motor(&J60Motor_CAN2[0], J60Motor_StandUpData_CAN2[0] + angle[2][1], 0, 0, 100, 5, PositionMode);
      HAL_Delay(1);
      RunJ60Motor(&J60Motor_CAN2[1], J60Motor_StandUpData_CAN2[1] - angle[2][0], 0, 0, 100, 5, PositionMode);
      HAL_Delay(1);

      // Leg_cyloid(&t, &angle[1][0], &angle[1][1], 1);
      // RunJ60Motor(&J60Motor_CAN1[2], J60Motor_StandUpData_CAN1[2] + angle[1][1], 0, 0, 100, 5, PositionMode);
      // HAL_Delay(1);
      // RunJ60Motor(&J60Motor_CAN1[3], J60Motor_StandUpData_CAN1[3] - angle[1][0], 0, 0, 100, 5, PositionMode);
      // HAL_Delay(1);

      // Leg_cyloid(&t, &angle[3][0], &angle[3][1], 1);
      // RunJ60Motor(&J60Motor_CAN2[2], J60Motor_StandUpData_CAN2[2] - angle[3][1], 0, 0, 100, 5, PositionMode);
      // HAL_Delay(1);
      // RunJ60Motor(&J60Motor_CAN2[3], J60Motor_StandUpData_CAN2[3] + angle[3][0], 0, 0, 100, 5, PositionMode);
      // HAL_Delay(1);

      t += speed;
    }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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

#ifdef  USE_FULL_ASSERT
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

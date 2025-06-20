/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "dma.h"
#include "fatfs.h"
#include "sdio.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "DeepJ60_Motor.h"
#include "leg_control.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define SPI_BUFFER_SIZE 15
#define CAN_BUFFER_SIZE 15

CAN_TxHeaderTypeDef TxHeader;
int test = 0;
typedef struct {
    uint32_t current_angle;
    uint32_t current_angular_velocity;
    uint16_t current_torque;
    uint8_t temperature_flag;
    uint8_t current_temperature;
} J60_MotorReturnInfo;
J60_MotorReturnInfo motor1_info;
J60_MotorReturnInfo motor2_info;
J60_MotorReturnInfo motor3_info;
J60_MotorReturnInfo motor4_info;

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
uint8_t spi_rx_flag = 0;


float total_time = 1000;
float angle[4][2];
float pos[4][2];
float t = 0;
float last_t = 0;
float speed = 0.01;

float x[2] = {0};
float y[2] = {0};


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
motor_data spi_motor_data;
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

  float Kp = 0;
  float Kd = 0;
  enum MotorMode motor_mode;
  
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
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_SPI1_Init();
  MX_SDIO_SD_Init();
  MX_TIM2_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */

  HAL_Delay(4000);



  EnableJ60Motor(&J60Motor_CAN1[2], 3, 1);  // rf out
  EnableJ60Motor(&J60Motor_CAN1[3], 4, 1);  // rf in
  EnableJ60Motor(&J60Motor_CAN2[2], 3, 2);  // lb out
  EnableJ60Motor(&J60Motor_CAN2[3], 4, 2);  // lb in

  HAL_Delay(1);
  RunJ60Motor(&J60Motor_CAN1[2], J60Motor_StandUpData_CAN1[2], 0, 0, 20, 4, PositionMode);
  HAL_Delay(1);
  RunJ60Motor(&J60Motor_CAN1[3], J60Motor_StandUpData_CAN1[3], 0, 0, 20, 4, PositionMode);
  HAL_Delay(1);
  RunJ60Motor(&J60Motor_CAN2[2], J60Motor_StandUpData_CAN2[2], 0, 0, 20, 4, PositionMode);
  HAL_Delay(1);
  RunJ60Motor(&J60Motor_CAN2[3], J60Motor_StandUpData_CAN2[3], 0, 0, 20, 4, PositionMode);
  
//  HAL_Delay(1);
//  RunJ60Motor(&J60Motor_CAN1[2], 0, 0, 0, 0, 0, ZeroTorqueMode);
//  HAL_Delay(1);
//  RunJ60Motor(&J60Motor_CAN1[3], 0, 0, 0, 0, 0, ZeroTorqueMode);
//  HAL_Delay(1);
//  RunJ60Motor(&J60Motor_CAN2[2], 0, 0, 0, 0, 0, ZeroTorqueMode);
//  HAL_Delay(1);
//  RunJ60Motor(&J60Motor_CAN2[3], 0, 0, 0, 0, 0, ZeroTorqueMode);

  HAL_Delay(2000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  HAL_Delay(1);
//  RunJ60Motor(&J60Motor_CAN1[2], 0, 0, 0, 0, 0, ZeroTorqueMode);
//  HAL_Delay(1);
//  RunJ60Motor(&J60Motor_CAN1[3], 0, 0, 0, 0, 0, ZeroTorqueMode);
//  HAL_Delay(1);
//  RunJ60Motor(&J60Motor_CAN2[2], 0, 0, 0, 0, 0, ZeroTorqueMode);
//  HAL_Delay(1);
//  RunJ60Motor(&J60Motor_CAN2[3], 0, 0, 0, 0, 0, ZeroTorqueMode);

    if(spi_rx_flag == NORMAL_COMMAND){
      Kp = spi_motor_data.real_motor_data[12];
      Kd = spi_motor_data.real_motor_data[13];
      motor_mode = (enum MotorMode)((int)spi_motor_data.real_motor_data[14]);

      if (motor_mode == PositionMode || motor_mode == PositionTorqueMode) {
        RunJ60Motor(&J60Motor_CAN1[2], 
                    J60Motor_StandUpData_CAN1[2] + spi_motor_data.real_motor_data[1], 
                    spi_motor_data.real_motor_data[5], 
                    spi_motor_data.real_motor_data[9], 
                    Kp, 
                    Kd, 
                    motor_mode);
        HAL_Delay(1);
        RunJ60Motor(&J60Motor_CAN1[3], 
                    J60Motor_StandUpData_CAN1[3] - spi_motor_data.real_motor_data[0], 
                    spi_motor_data.real_motor_data[4], 
                    spi_motor_data.real_motor_data[8], 
                    Kp, 
                    Kd, 
                    motor_mode);
        HAL_Delay(1);
        RunJ60Motor(&J60Motor_CAN2[2], 
                    J60Motor_StandUpData_CAN2[2] - spi_motor_data.real_motor_data[3], 
                    spi_motor_data.real_motor_data[7], 
                    spi_motor_data.real_motor_data[11], 
                    Kp, 
                    Kd, 
                    motor_mode);
        HAL_Delay(1);
        RunJ60Motor(&J60Motor_CAN2[3], 
                    J60Motor_StandUpData_CAN2[3] + spi_motor_data.real_motor_data[2], 
                    spi_motor_data.real_motor_data[6], 
                    spi_motor_data.real_motor_data[10], 
                    Kp, 
                    Kd, 
                    motor_mode);
      }
      else {
        RunJ60Motor(&J60Motor_CAN1[2], 
                    0,
                    spi_motor_data.real_motor_data[5], 
                    spi_motor_data.real_motor_data[9], 
                    Kp, 
                    Kd, 
                    motor_mode);
        HAL_Delay(1);
        RunJ60Motor(&J60Motor_CAN1[3], 
                    0, 
                    spi_motor_data.real_motor_data[4], 
                    spi_motor_data.real_motor_data[8], 
                    Kp, 
                    Kd, 
                    motor_mode);
        HAL_Delay(1);
        RunJ60Motor(&J60Motor_CAN2[2], 
                    0, 
                    spi_motor_data.real_motor_data[7], 
                    spi_motor_data.real_motor_data[11], 
                    Kp, 
                    Kd, 
                    motor_mode);
        HAL_Delay(1);
        RunJ60Motor(&J60Motor_CAN2[3], 
                    0, 
                    spi_motor_data.real_motor_data[6], 
                    spi_motor_data.real_motor_data[10], 
                    Kp, 
                    Kd, 
                    motor_mode);
      }

      spi_rx_flag = NO_COMMAND;
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
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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

//void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
//{
//    if (hspi == &hspi1)
//    {
//        // transfer_spi_to_can();
//        if (HAL_SPI_Receive_IT(&hspi1, spi_motor_data.rx_data, 60) != HAL_OK)
//        {
//            Error_Handler();
//        }
//    }
//}

//void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
//{
//    CAN_RxHeaderTypeDef RxHeader;
//    uint8_t RxData[8];
//
//    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
//    {
//        switch (RxHeader.StdId)
//        {
//        case MOTOR1_CAN_ID:
//            // 处理电机 1 返回的数�?
//            for (int i = 0; i < RxHeader.DLC; i++)
//            {
//              motor1_info.current_angle = ((uint32_t)RxData[0] << 12) | ((uint32_t)RxData[1] << 4) | ((RxData[2] & 0xF0) >> 4);                 //解析当前角度
//              motor1_info.current_angular_velocity = (((uint32_t)(RxData[2] & 0x0F) << 16) | ((uint32_t)RxData[3] << 8) | RxData[4]);                 // 解析当前角�?�度
//              motor1_info.current_torque = ((uint16_t)(RxData[5] << 8) | RxData[6]);            // 解析当前扭矩
//              motor1_info.temperature_flag = (RxData[7] & 0x80) >> 7;            // 解析温度标志�?
//              motor1_info.current_temperature = RxData[7] & 0x7F;            // 解析当前温度
//            }
//            break;
//        case MOTOR2_CAN_ID:
//            // 处理电机 2 返回的数�?
//            for (int i = 0; i < RxHeader.DLC; i++)
//            {
//              motor2_info.current_angle = ((uint32_t)RxData[0] << 12) | ((uint32_t)RxData[1] << 4) | ((RxData[2] & 0xF0) >> 4);                 //解析当前角度
//              motor2_info.current_angular_velocity = (((uint32_t)(RxData[2] & 0x0F) << 16) | ((uint32_t)RxData[3] << 8) | RxData[4]);                 // 解析当前角�?�度
//              motor2_info.current_torque = ((uint16_t)(RxData[5] << 8) | RxData[6]);            // 解析当前扭矩
//              motor2_info.temperature_flag = (RxData[7] & 0x80) >> 7;            // 解析温度标志�?
//              motor2_info.current_temperature = RxData[7] & 0x7F;            // 解析当前温度
//            }
//            break;
//        case MOTOR3_CAN_ID:
//            // 处理电机 3 返回的数�?
//            for (int i = 0; i < RxHeader.DLC; i++)
//            {
//              motor3_info.current_angle = ((uint32_t)RxData[0] << 12) | ((uint32_t)RxData[1] << 4) | ((RxData[2] & 0xF0) >> 4);                 //解析当前角度
//              motor3_info.current_angular_velocity = (((uint32_t)(RxData[2] & 0x0F) << 16) | ((uint32_t)RxData[3] << 8) | RxData[4]);                 // 解析当前角�?�度
//              motor3_info.current_torque = ((uint16_t)(RxData[5] << 8) | RxData[6]);            // 解析当前扭矩
//              motor3_info.temperature_flag = (RxData[7] & 0x80) >> 7;            // 解析温度标志�?
//              motor3_info.current_temperature = RxData[7] & 0x7F;            // 解析当前温度
//            }
//            break;
//        case MOTOR4_CAN_ID:
//            // 处理电机 4 返回的数�?
//            for (int i = 0; i < RxHeader.DLC; i++)
//            {
//              motor4_info.current_angle = ((uint32_t)RxData[0] << 12) | ((uint32_t)RxData[1] << 4) | ((RxData[2] & 0xF0) >> 4);                 //解析当前角度
//              motor4_info.current_angular_velocity = (((uint32_t)(RxData[2] & 0x0F) << 16) | ((uint32_t)RxData[3] << 8) | RxData[4]);                 // 解析当前角�?�度
//              motor4_info.current_torque = ((uint16_t)(RxData[5] << 8) | RxData[6]);            // 解析当前扭矩
//              motor4_info.temperature_flag = (RxData[7] & 0x80) >> 7;            // 解析温度标志�?
//              motor4_info.current_temperature = RxData[7] & 0x7F;            // 解析当前温度
//            }
//            break;
//        default:
//            break;
//        }
//    }
//}

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

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "GaitController.h"
#include "Handle.h"
#include "imu.h"
#include "Camera.h"
#include "nrf.h"
#include "tim.h"
#include "math.h"
#include "param.h"
#include "foot_attitude_calculation.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define START_ACTION 2
#define END_ACTION 1
#define START_JUMP_1 3

#define INIT_TROT_LENGTH 0.2f
#define MAX_TROT_LENGTH 0.3f

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

osThreadId_t TaskHandle = NULL;
int is_stack_overflow = 0;

float trot_length = INIT_TROT_LENGTH;

int tim_callback = 0;

//uint8_t rx_cmd[HANDLE_DATA_SIZE];

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for TrotForward */
osThreadId_t TrotForwardHandle;
const osThreadAttr_t TrotForward_attributes = {
  .name = "TrotForward",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for TrotBack */
osThreadId_t TrotBackHandle;
const osThreadAttr_t TrotBack_attributes = {
  .name = "TrotBack",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for RotateLeft */
osThreadId_t RotateLeftHandle;
const osThreadAttr_t RotateLeft_attributes = {
  .name = "RotateLeft",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for RotateRight */
osThreadId_t RotateRightHandle;
const osThreadAttr_t RotateRight_attributes = {
  .name = "RotateRight",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for MoveLeft */
osThreadId_t MoveLeftHandle;
const osThreadAttr_t MoveLeft_attributes = {
  .name = "MoveLeft",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for MoveRight */
osThreadId_t MoveRightHandle;
const osThreadAttr_t MoveRight_attributes = {
  .name = "MoveRight",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for JumpUp */
osThreadId_t JumpUpHandle;
const osThreadAttr_t JumpUp_attributes = {
  .name = "JumpUp",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for JumpForward */
osThreadId_t JumpForwardHandle;
const osThreadAttr_t JumpForward_attributes = {
  .name = "JumpForward",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for WalkSlope */
osThreadId_t WalkSlopeHandle;
const osThreadAttr_t WalkSlope_attributes = {
  .name = "WalkSlope",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for WalkSlope_LR */
osThreadId_t WalkSlope_LRHandle;
const osThreadAttr_t WalkSlope_LR_attributes = {
  .name = "WalkSlope_LR",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Stand */
osThreadId_t StandHandle;
const osThreadAttr_t Stand_attributes = {
  .name = "Stand",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ParseHandle */
osThreadId_t ParseHandleHandle;
const osThreadAttr_t ParseHandle_attributes = {
  .name = "ParseHandle",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for ParseIMU */
osThreadId_t ParseIMUHandle;
const osThreadAttr_t ParseIMU_attributes = {
  .name = "ParseIMU",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for NotifyAction */
osThreadId_t NotifyActionHandle;
const osThreadAttr_t NotifyAction_attributes = {
  .name = "NotifyAction",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for ParseCamera */
osThreadId_t ParseCameraHandle;
const osThreadAttr_t ParseCamera_attributes = {
  .name = "ParseCamera",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for TurnLeft */
osThreadId_t TurnLeftHandle;
const osThreadAttr_t TurnLeft_attributes = {
  .name = "TurnLeft",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for TurnRight */
osThreadId_t TurnRightHandle;
const osThreadAttr_t TurnRight_attributes = {
  .name = "TurnRight",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void TrotForwardTask(void *argument);
void TrotBackTask(void *argument);
void RotateLeftTask(void *argument);
void RotateRightTask(void *argument);
void MoveLeftTask(void *argument);
void MoveRightTask(void *argument);
void JumpUpTask(void *argument);
void JumpForwardTask(void *argument);
void WalkSlopeTask(void *argument);
void WalkSlopeLRTask(void *argument);
void StandTask(void *argument);
void ParseHandleTask(void *argument);
void ParseIMUTask(void *argument);
void NotifyActionTask(void *argument);
void ParseCameraTask(void *argument);
void TurnLeftTask(void *argument);
void TurnRightTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
   is_stack_overflow = 1;
   while (1);
}
/* USER CODE END 4 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of TrotForward */
  TrotForwardHandle = osThreadNew(TrotForwardTask, NULL, &TrotForward_attributes);

  /* creation of TrotBack */
  TrotBackHandle = osThreadNew(TrotBackTask, NULL, &TrotBack_attributes);

  /* creation of RotateLeft */
  RotateLeftHandle = osThreadNew(RotateLeftTask, NULL, &RotateLeft_attributes);

  /* creation of RotateRight */
  RotateRightHandle = osThreadNew(RotateRightTask, NULL, &RotateRight_attributes);

  /* creation of MoveLeft */
  MoveLeftHandle = osThreadNew(MoveLeftTask, NULL, &MoveLeft_attributes);

  /* creation of MoveRight */
  MoveRightHandle = osThreadNew(MoveRightTask, NULL, &MoveRight_attributes);

  /* creation of JumpUp */
  JumpUpHandle = osThreadNew(JumpUpTask, NULL, &JumpUp_attributes);

  /* creation of JumpForward */
  JumpForwardHandle = osThreadNew(JumpForwardTask, NULL, &JumpForward_attributes);

  /* creation of WalkSlope */
  WalkSlopeHandle = osThreadNew(WalkSlopeTask, NULL, &WalkSlope_attributes);

  /* creation of WalkSlope_LR */
  WalkSlope_LRHandle = osThreadNew(WalkSlopeLRTask, NULL, &WalkSlope_LR_attributes);

  /* creation of Stand */
  StandHandle = osThreadNew(StandTask, NULL, &Stand_attributes);

  /* creation of ParseHandle */
  ParseHandleHandle = osThreadNew(ParseHandleTask, NULL, &ParseHandle_attributes);

  /* creation of ParseIMU */
  ParseIMUHandle = osThreadNew(ParseIMUTask, NULL, &ParseIMU_attributes);

  /* creation of NotifyAction */
  NotifyActionHandle = osThreadNew(NotifyActionTask, NULL, &NotifyAction_attributes);

  /* creation of ParseCamera */
  ParseCameraHandle = osThreadNew(ParseCameraTask, NULL, &ParseCamera_attributes);

  /* creation of TurnLeft */
  TurnLeftHandle = osThreadNew(TurnLeftTask, NULL, &TurnLeft_attributes);

  /* creation of TurnRight */
  TurnRightHandle = osThreadNew(TurnRightTask, NULL, &TurnRight_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_TrotForwardTask */
/**
* @brief Function implementing the TrotForward thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TrotForwardTask */
void TrotForwardTask(void *argument)
{
  /* USER CODE BEGIN TrotForwardTask */

  uint32_t notify_value = 0;
  float coef = trot_param.length / 125;
  float coef_turn = 0.1 / 125;
  float trot_height = 0.03;

  /* Infinite loop */
  for(;;)
  {
    if (xTaskNotifyWait(0, 0, &notify_value, 0) == pdTRUE) {
      if (notify_value == START_ACTION) {
        if (handle_command[0] == TROT_FORWARD_CMD || handle_command[0] == JUMP_UP_CMD) {
          if (trot_controller.trot_state == EndTrot) {
            trot_controller.trot_state = PreTrot;
            trot_controller.trot_direction = Forward;
            t = 0;
          }
        }
        else {
          if (turn_controller.turn_state == EndTurn) {
            turn_controller.turn_state = PreTurn;
            t = 0;
          }
        }
      }
      else if (notify_value == END_ACTION) {
        if (turn_controller.turn_state == Turning || trot_controller.trot_state == Trotting) {
          isStop = NEED_TO_STOP;
        }
      }

      if (turn_controller.turn_state != EndTurn) {

        trot_length = coef * abs(handle_command[2] - 125);

        if (handle_command[3] - 125 > 0) {
          turn_controller.turn_angular_direction = TurnRight;
          left_length = trot_length + coef_turn * abs(handle_command[3] - 125) / 2;
          right_length = trot_length - coef_turn * abs(handle_command[3] - 125) / 2;
          Turn_FSM(&turn_controller, right_length, left_length, 0.03, robot_height);
        }
        else if (handle_command[3] - 125 < 0) {
          turn_controller.turn_angular_direction = TurnLeft;
          left_length = trot_length - coef_turn * abs(handle_command[3] - 125) / 2;
          right_length = trot_length + coef_turn * abs(handle_command[3] - 125) / 2;
          Turn_FSM(&turn_controller, left_length, right_length, 0.03, robot_height);
        }
        else {
          Turn_FSM(&turn_controller, trot_length, trot_length, 0.03, robot_height);
        }

        if (turn_controller.turn_state != EndTurn) {
          __HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);
          __HAL_TIM_SET_COUNTER(&htim2, 0);
          HAL_TIM_Base_Start_IT(&htim2);
        }
      }
      if (trot_controller.trot_state != EndTrot) {
        
        if (robot_height < 0.2096) {
          trot_length = trot_param.length / 0.2096 * robot_height;
        }
        else {
          trot_length = trot_param.length;
        }
        trot_height = trot_param.height / 0.2096 * robot_height;
        Trot_FSM(&trot_controller, trot_height, trot_length, robot_height);
        // Trot_FSM(&trot_controller, 0.03, (0.0024 * abs(handle_command[2] - 125)), robot_height);

        if (trot_controller.trot_state != EndTrot) {
          __HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);
          __HAL_TIM_SET_COUNTER(&htim2, 0);
          HAL_TIM_Base_Start_IT(&htim2);
        }
      }

      notify_value = 0;
    }

    osDelay(1);
  }
  /* USER CODE END TrotForwardTask */
}

/* USER CODE BEGIN Header_TrotBackTask */
/**
* @brief Function implementing the TrotBack thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TrotBackTask */
void TrotBackTask(void *argument)
{
  /* USER CODE BEGIN TrotBackTask */

  uint32_t notify_value = 0;
  float coef = trot_param.length / 125;

  /* Infinite loop */
  for(;;)
  {
    if (xTaskNotifyWait(0, 0, &notify_value, 0) == pdTRUE) {
      if (notify_value == START_ACTION) {
        if (trot_controller.trot_state == EndTrot) {
          trot_controller.trot_state = PreTrot;
          trot_controller.trot_direction = Back;
          t = 0;
        }
      }
      else if (notify_value == END_ACTION) {
        if (trot_controller.trot_state == Trotting) {
          isStop = NEED_TO_STOP;
        }
      }

      if (trot_controller.trot_state != EndTrot) {
        if (handle_command[0] == TROT_BACK_CMD || handle_command[0] == JUMP_FORWARD_CMD) {
          Trot_FSM(&trot_controller, trot_param.height, trot_param.length, robot_height);
        }
        else {
          Trot_FSM(&trot_controller, trot_param.height, (coef * abs(handle_command[2] - 125)), robot_height);
        }
        // Trot_FSM(&trot_controller, 0.03, trot_length, robot_height);
        // Trot_FSM(&trot_controller, 0.03, (coef * abs(handle_command[2] - 125)), robot_height);

        if (trot_controller.trot_state != EndTrot) {
          __HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);
          __HAL_TIM_SET_COUNTER(&htim2, 0);
          HAL_TIM_Base_Start_IT(&htim2);
        }
      }

      notify_value = 0;
    }

    osDelay(1);
  }
  /* USER CODE END TrotBackTask */
}

/* USER CODE BEGIN Header_RotateLeftTask */
/**
* @brief Function implementing the RotateLeft thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_RotateLeftTask */
void RotateLeftTask(void *argument)
{
  /* USER CODE BEGIN RotateLeftTask */

  uint32_t notify_value = 0;

  /* Infinite loop */
  for(;;)
  {
    if (xTaskNotifyWait(0, 0, &notify_value, 0) == pdTRUE) {
      if (notify_value == START_ACTION) {
        if (rotate_controller.rotate_state == EndRotate) {
          rotate_controller.rotate_state = PreRotate;
          rotate_controller.rotate_direction = Left;
          t = 0;
        }
      }
      else if (notify_value == END_ACTION) {
        if (rotate_controller.rotate_state == Rotating) {
          isStop = NEED_TO_STOP;
        }
      }

      if (rotate_controller.rotate_state != EndRotate) {
        Rotate_FSM(&rotate_controller, rotate_param.height, rotate_param.length, robot_height);

        if (rotate_controller.rotate_state != EndRotate) {
          __HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);
          __HAL_TIM_SET_COUNTER(&htim2, 0);
          HAL_TIM_Base_Start_IT(&htim2);
        }
      }

      notify_value = 0;
    }

    osDelay(1);
  }
  /* USER CODE END RotateLeftTask */
}

/* USER CODE BEGIN Header_RotateRightTask */
/**
* @brief Function implementing the RotateRight thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_RotateRightTask */
void RotateRightTask(void *argument)
{
  /* USER CODE BEGIN RotateRightTask */

  uint32_t notify_value = 0;

  /* Infinite loop */
  for(;;)
  {
    if (xTaskNotifyWait(0, 0, &notify_value, 0) == pdTRUE) {
      if (notify_value == START_ACTION) {
        if (rotate_controller.rotate_state == EndRotate) {
          rotate_controller.rotate_state = PreRotate;
          rotate_controller.rotate_direction = Right;
          t = 0;
        }
      }
      else if (notify_value == END_ACTION) {
        if (rotate_controller.rotate_state == Rotating) {
          isStop = NEED_TO_STOP;
        }
      }

      if (rotate_controller.rotate_state != EndRotate) {
        Rotate_FSM(&rotate_controller, rotate_param.height, rotate_param.length, robot_height);

        if (rotate_controller.rotate_state != EndRotate) {
          __HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);
          __HAL_TIM_SET_COUNTER(&htim2, 0);
          HAL_TIM_Base_Start_IT(&htim2);
        }
      }

      notify_value = 0;
    }

    osDelay(1);
  }
  /* USER CODE END RotateRightTask */
}

/* USER CODE BEGIN Header_MoveLeftTask */
/**
* @brief Function implementing the MoveLeft thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MoveLeftTask */
void MoveLeftTask(void *argument)
{
  /* USER CODE BEGIN MoveLeftTask */

  uint32_t notify_value = 0;

  /* Infinite loop */
  for(;;)
  {
    if (xTaskNotifyWait(0, 0, &notify_value, 0) == pdTRUE) {
      if (notify_value == START_ACTION) {
        // if (turn_controller.turn_state == EndTurn) {
        //   turn_controller.turn_state = PreTurn;
        //   turn_controller.turn_angular_direction = TurnLeft;
        //   t = 0;
        // }
        if (move_controller.move_state == EndMove) {
          move_controller.move_state = PreMove;
          move_controller.move_direction = MoveLeft;
          t = 0;
        }
      }
      else if (notify_value == END_ACTION) {
        // if (turn_controller.turn_state == Turning) {
        //   isStop = NEED_TO_STOP;
        // }
        if (move_controller.move_state == Moving) {
          isStop = NEED_TO_STOP;
        }
      }

      // if (turn_controller.turn_state != EndTurn) {
      //   Turn_FSM(&turn_controller, 0.04, 0.1, 0.03, robot_height);

      //   if (turn_controller.turn_state != EndTurn) {
      //     __HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);
      //     __HAL_TIM_SET_COUNTER(&htim2, 0);
      //     HAL_TIM_Base_Start_IT(&htim2);
      //   }
      // }
      
      if (move_controller.move_state != EndMove) {
        HorizontalMove_FSM(&move_controller, robot_height + 0.03, robot_height - 0.03, robot_height, 0.04);

        if (move_controller.move_state != EndMove) {
          __HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);
          __HAL_TIM_SET_COUNTER(&htim2, 0);
          HAL_TIM_Base_Start_IT(&htim2);
        }
      }

      notify_value = 0;
    }

    osDelay(1);
  }
  /* USER CODE END MoveLeftTask */
}

/* USER CODE BEGIN Header_MoveRightTask */
/**
* @brief Function implementing the MoveRight thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MoveRightTask */
void MoveRightTask(void *argument)
{
  /* USER CODE BEGIN MoveRightTask */

  uint32_t notify_value = 0;

  /* Infinite loop */
  for(;;)
  {
    if (xTaskNotifyWait(0, 0, &notify_value, 0) == pdTRUE) {
      if (notify_value == START_ACTION) {
        // if (turn_controller.turn_state == EndTurn) {
        //   turn_controller.turn_state = PreTurn;
        //   turn_controller.turn_angular_direction = TurnRight;
        //   t = 0;
        // }
        if (move_controller.move_state == EndMove) {
          move_controller.move_state = PreMove;
          move_controller.move_direction = MoveRight;
          t = 0;
        }
      }
      else if (notify_value == END_ACTION) {
        // if (turn_controller.turn_state == Turning) {
        //   isStop = NEED_TO_STOP;
        // }
        if (move_controller.move_state == Moving) {
          isStop = NEED_TO_STOP;
        }
      }

      // if (turn_controller.turn_state != EndTurn) {
      //   Turn_FSM(&turn_controller, 0.04, 0.1, 0.03, robot_height);

      //   if (turn_controller.turn_state != EndTurn) {
      //     __HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);
      //     __HAL_TIM_SET_COUNTER(&htim2, 0);
      //     HAL_TIM_Base_Start_IT(&htim2);
      //   }
      // }

      if (move_controller.move_state != EndMove) {
        HorizontalMove_FSM(&move_controller, robot_height + 0.03, robot_height - 0.03, robot_height, 0.04);

        if (move_controller.move_state != EndMove) {
          __HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);
          __HAL_TIM_SET_COUNTER(&htim2, 0);
          HAL_TIM_Base_Start_IT(&htim2);
        }
      }

      notify_value = 0;
    }

    osDelay(1);
  }
  /* USER CODE END MoveRightTask */
}

/* USER CODE BEGIN Header_JumpUpTask */
/**
* @brief Function implementing the JumpUp thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_JumpUpTask */
void JumpUpTask(void *argument)
{
  /* USER CODE BEGIN JumpUpTask */

  const TickType_t freq = pdMS_TO_TICKS(10);
  TickType_t last_time = xTaskGetTickCount();
  uint32_t notify_value = 0;

  /* Infinite loop */
  for(;;)
  {
    if (xTaskNotifyWait(0, 0, &notify_value, 0) == pdTRUE) {
      if (notify_value == START_ACTION) {
        if (jump_up_controller.jump_state == EndJump) {
          jump_up_controller.jump_state = Squat;
          t = 0;
        }
      }
    }

    if (jump_up_controller.jump_state != EndJump) {
      JumpUp_FSM(&jump_up_controller);

      if (jump_up_controller.jump_state != EndJump && jump_up_controller.jump_state != Land && jump_up_controller.jump_state != JumpUp) {
        vTaskDelayUntil(&last_time, freq);
        t += JUMP_T;
      }
    }

    osDelay(1);
  }
  /* USER CODE END JumpUpTask */
}

/* USER CODE BEGIN Header_JumpForwardTask */
/**
* @brief Function implementing the JumpForward thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_JumpForwardTask */
void JumpForwardTask(void *argument)
{
  /* USER CODE BEGIN JumpForwardTask */

  const TickType_t freq = pdMS_TO_TICKS(10);
  TickType_t last_time = xTaskGetTickCount();
  uint32_t notify_value = 0;

  /* Infinite loop */
  for(;;)
  {
    if (xTaskNotifyWait(0, 0, &notify_value, 0) == pdTRUE) {
      if (notify_value == START_ACTION) {
        if (jump_forward_controller.jump_state == EndJump) {
          jump_forward_controller.jump_state = Squat;
          t = 0;
        }
        // else if (jump_forward_controller.jump_state == Squat) {
        //   jump_forward_controller.jump_state = JumpUp;
        //   t = 0;
        // }
      }
      // else if (notify_value == START_JUMP_1) {
      //   if (jump_forward_controller.jump_state == EndJump) {
      //     jump_forward_controller.jump_state = Squat;
      //     t = 0;
      //   }
      // }
    }

    if (jump_forward_controller.jump_state != EndJump) {
      JumpForward_FSM(&jump_forward_controller);

      if (jump_forward_controller.jump_state != EndJump && jump_forward_controller.jump_state != Land && jump_forward_controller.jump_state != JumpUp) {
        vTaskDelayUntil(&last_time, freq);
        t += JUMP_T;
      }
    }

    osDelay(1);
  }
  /* USER CODE END JumpForwardTask */
}

/* USER CODE BEGIN Header_WalkSlopeTask */
/**
* @brief Function implementing the WalkSlope thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_WalkSlopeTask */
void WalkSlopeTask(void *argument)
{
  /* USER CODE BEGIN WalkSlopeTask */

  uint32_t notify_value = 0;

  /* Infinite loop */
  for(;;)
  {
    if (xTaskNotifyWait(0, 0, &notify_value, 0) == pdTRUE) {
      if (notify_value == START_ACTION) {
        if (walk_slope_controller.trot_state == EndTrot) {
          walk_slope_controller.trot_state = PreTrot;
          walk_slope_controller.trot_direction = Forward;
          t = 0;
        }
      }
      else if (notify_value == END_ACTION) {
        if (walk_slope_controller.trot_state == Trotting) {
          isStop = NEED_TO_STOP;
        }
      }

      if (walk_slope_controller.trot_state != EndTrot) {
        WalkSlope_FSM(&walk_slope_controller, (1.0 / 3), 0.4, robot_height, 0.12, 0.04);

        if (walk_slope_controller.trot_state != EndTrot) {
          __HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);
          __HAL_TIM_SET_COUNTER(&htim2, 0);
          HAL_TIM_Base_Start_IT(&htim2);
        }
      }

      notify_value = 0;
    }

    osDelay(1);
  }
  /* USER CODE END WalkSlopeTask */
}

/* USER CODE BEGIN Header_WalkSlopeLRTask */
/**
* @brief Function implementing the WalkSlope_LR thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_WalkSlopeLRTask */
void WalkSlopeLRTask(void *argument)
{
  /* USER CODE BEGIN WalkSlopeLRTask */

  uint32_t notify_value = 0;

  /* Infinite loop */
  for(;;)
  {
    if (xTaskNotifyWait(0, 0, &notify_value, 0) == pdTRUE) {
      if (notify_value == START_ACTION) {
        if (walk_LR_slope_controller.trot_state == EndTrot) {
          walk_LR_slope_controller.trot_state = PreTrot;
          walk_LR_slope_controller.trot_direction = Forward;
          t = 0;
        }
      }
      else if (notify_value == END_ACTION) {
        if (walk_LR_slope_controller.trot_state == Trotting) {
          isStop = NEED_TO_STOP;
        }
      }

      if (walk_LR_slope_controller.trot_state != EndTrot) {
        WalkSlope_LR_FSM(&walk_LR_slope_controller, 0.268, 0.4, robot_height, 0.12, 0.09378);

        if (walk_LR_slope_controller.trot_state != EndTrot) {
          __HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);
          __HAL_TIM_SET_COUNTER(&htim2, 0);
          HAL_TIM_Base_Start_IT(&htim2);
        }
      }

      notify_value = 0;
    }

    osDelay(1);
  }
  /* USER CODE END WalkSlopeLRTask */
}

/* USER CODE BEGIN Header_StandTask */
/**
* @brief Function implementing the Stand thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StandTask */
void StandTask(void *argument)
{
  /* USER CODE BEGIN StandTask */
  /* Infinite loop */
  for(;;)
  {
    if (trot_controller.trot_state == EndTrot && rotate_controller.rotate_state == EndRotate &&
        jump_up_controller.jump_state == EndJump && jump_forward_controller.jump_state == EndJump &&
        turn_controller.turn_state == EndTurn && walk_slope_controller.trot_state == EndTrot &&
        walk_LR_slope_controller.trot_state == EndTrot && move_controller.move_state == EndMove) {

      if (isSlope == NO_SLOPE) {
        Stand();
		  //trot_length = 0.2;
      }
      else if (isSlope == SLOPE) {
        Stand_on_slope(tan_slope_theta);
      }
      else if (isSlope == SLOPE_LR) {
        Stand_on_LR_slope(tan_LR_slope_theta);
      }
    }

    osDelay(10);
  }
  /* USER CODE END StandTask */
}

/* USER CODE BEGIN Header_ParseHandleTask */
/**
* @brief Function implementing the ParseHandle thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ParseHandleTask */
void ParseHandleTask(void *argument)
{
  /* USER CODE BEGIN ParseHandleTask */

  uint8_t rx_cmd[HANDLE_DATA_SIZE];
  uint8_t last_handle_command[CMD_SIZE] = {0};

  /* Infinite loop */
  for(;;)
  {
    if (xQueueReceive(cmd_queue, rx_cmd, portMAX_DELAY) == pdPASS) {
      if (CheckRxData(rx_cmd) == CMD_NORMAL) {
        ParseHandle(rx_cmd, handle_command);
        if (
          (last_handle_command[0] == STOP_CMD && handle_command[0] != STOP_CMD && 
          handle_command[0] != JUMP_UP_CMD && handle_command[0] != JUMP_FORWARD_CMD) ||
          (last_handle_command[2] == 125 && handle_command[2] != 125) ||
          (handle_command[1] == MODE_5 && last_handle_command[0] == STOP_CMD && handle_command[0] != STOP_CMD)) {
          
		      __HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);
          __HAL_TIM_SET_COUNTER(&htim2, 0);
          HAL_TIM_Base_Start_IT(&htim2);
        }

        memcpy(last_handle_command, handle_command, sizeof(handle_command));
      }
    }

    osDelay(1);
  }
  /* USER CODE END ParseHandleTask */
}

/* USER CODE BEGIN Header_ParseIMUTask */
/**
* @brief Function implementing the ParseIMU thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ParseIMUTask */
void ParseIMUTask(void *argument)
{
  /* USER CODE BEGIN ParseIMUTask */

  /* Infinite loop */
  for(;;)
  {
    // for(uint8_t i = 0; i < 23; i++) {
		// 	HAL_UART_Receive(&huart2, imu_rx_data + i, 1, 10);
		// }

    // Q_Value.x.val_32bit=u8_to_u32(imu_rx_data[7],imu_rx_data[6],imu_rx_data[5],imu_rx_data[4]);
		// Q_Value.y.val_32bit=u8_to_u32(imu_rx_data[11],imu_rx_data[10],imu_rx_data[9],imu_rx_data[8]);
		// Q_Value.z.val_32bit=u8_to_u32(imu_rx_data[15],imu_rx_data[14],imu_rx_data[13],imu_rx_data[12]);
		// Q_Value.w.val_32bit=u8_to_u32(imu_rx_data[19],imu_rx_data[18],imu_rx_data[17],imu_rx_data[16]);

		// IMU_quaterToEulerianAngles();

    // angle_to_position();

    osDelay(1);
  }
  /* USER CODE END ParseIMUTask */
}

/* USER CODE BEGIN Header_NotifyActionTask */
/**
* @brief Function implementing the NotifyAction thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_NotifyActionTask */
void NotifyActionTask(void *argument)
{
  /* USER CODE BEGIN NotifyActionTask */

  uint8_t last_handle_command[CMD_SIZE] = {0};

  /* Infinite loop */
  for(;;)
  {
    if (t != pre_t) {
      if (handle_command[1] == MODE_1) {  // 正常行走，旋�?
        if (handle_command[0] == TROT_FORWARD_CMD || handle_command[2] - 125 < 0) {  // 前进
          if (TaskHandle != TrotForwardHandle) {
            trot_param.length = 0.3;
            trot_param.height = 0.03;
            trot_param.Kp = 100;
            trot_param.Kd = 4;
          }
          TaskHandle = TrotForwardHandle;
          xTaskNotify((TaskHandle_t)TrotForwardHandle, START_ACTION, eSetValueWithoutOverwrite);
        }
        else if (handle_command[0] == TROT_BACK_CMD || handle_command[2] - 125 > 0) {  // 后退
          if (TaskHandle != TrotBackHandle) {
            trot_param.length = 0.3;
            trot_param.height = 0.03;
            trot_param.Kp = 100;
            trot_param.Kd = 4;
          }
          TaskHandle = TrotBackHandle;
          xTaskNotify((TaskHandle_t)TrotBackHandle, START_ACTION, eSetValueWithOverwrite);
        }
        else if (handle_command[0] == ROTATE_LEFT_CMD) {  // 左旋转
          if (TaskHandle != RotateLeftHandle) {
            rotate_param.length = 0.2;
            rotate_param.height = 0.03;
            rotate_param.Kp = 100;
            rotate_param.Kd = 4;
          }
          TaskHandle = RotateLeftHandle;
          xTaskNotify((TaskHandle_t)RotateLeftHandle, START_ACTION, eSetValueWithOverwrite);
        }
        else if (handle_command[0] == ROTATE_RIGHT_CMD) {  // 右旋转
          if (TaskHandle != RotateRightHandle) {
            rotate_param.length = 0.2;
            rotate_param.height = 0.03;
            rotate_param.Kp = 100;
            rotate_param.Kd = 4;
          }
          TaskHandle = RotateRightHandle;
          xTaskNotify((TaskHandle_t)RotateRightHandle, START_ACTION, eSetValueWithOverwrite);
        }
        else if (handle_command[0] == MOVE_LEFT_CMD) {  // 左转弯
          TaskHandle = TurnLeftHandle;
          xTaskNotify((TaskHandle_t)TurnLeftHandle, START_ACTION, eSetValueWithOverwrite);
        }
        else if (handle_command[0] == MOVE_RIGHT_CMD) {  // 右转弯
          TaskHandle = TurnRightHandle;
          xTaskNotify((TaskHandle_t)TurnRightHandle, START_ACTION, eSetValueWithOverwrite);
        }
        else if (handle_command[0] == STOP_CMD) {  // 停止
          if (TaskHandle != NULL) {
            xTaskNotify((TaskHandle_t)TaskHandle, END_ACTION, eSetValueWithOverwrite);
          }
        }
      }
      else if (handle_command[1] == MODE_2) {  // 斜坡、横移
        if (handle_command[0] == TROT_FORWARD_CMD) {  // 斜坡
          if (isSlope == SLOPE) {  // 前后斜坡
            TaskHandle = WalkSlopeHandle;
            xTaskNotify((TaskHandle_t)WalkSlopeHandle, START_ACTION, eSetValueWithoutOverwrite);
          }
          else if (isSlope == SLOPE_LR) {  // 左右斜坡
            TaskHandle = WalkSlope_LRHandle;
            xTaskNotify((TaskHandle_t)WalkSlope_LRHandle, START_ACTION, eSetValueWithoutOverwrite);
          }
          else {  // 正常
            trot_param.length = 0.3;
            trot_param.height = 0.03;
            trot_param.Kp = 100;
            trot_param.Kd = 4;
            TaskHandle = TrotForwardHandle;
            xTaskNotify((TaskHandle_t)TrotForwardHandle, START_ACTION, eSetValueWithoutOverwrite);
          }
        }
        else if (handle_command[2] - 125 < 0) {  // 前进摇杆
          trot_param.length = 0.3;
          trot_param.height = 0.03;
          trot_param.Kp = 100;
          trot_param.Kd = 4;
          TaskHandle = TrotForwardHandle;
          xTaskNotify((TaskHandle_t)TrotForwardHandle, START_ACTION, eSetValueWithoutOverwrite);
        }
        else if (handle_command[0] == TROT_BACK_CMD || handle_command[2] - 125 > 0) {  // 后�??
          trot_param.length = 0.3;
          trot_param.height = 0.03;
          trot_param.Kp = 100;
          trot_param.Kd = 4;
          TaskHandle = TrotBackHandle;
          xTaskNotify((TaskHandle_t)TrotBackHandle, START_ACTION, eSetValueWithOverwrite);
        }
        else if (handle_command[0] == ROTATE_LEFT_CMD) {  // 左横移
          TaskHandle = MoveLeftHandle;
          xTaskNotify((TaskHandle_t)MoveLeftHandle, START_ACTION, eSetValueWithOverwrite);
        }
        else if (handle_command[0] == ROTATE_RIGHT_CMD) {  // 右横移
          TaskHandle = MoveRightHandle;
          xTaskNotify((TaskHandle_t)MoveRightHandle, START_ACTION, eSetValueWithOverwrite);
        }
        else if (handle_command[0] == STOP_CMD) {  // 停止
          if (TaskHandle != NULL) {
            xTaskNotify((TaskHandle_t)TaskHandle, END_ACTION, eSetValueWithOverwrite);
          }
        }
      }
      else if (handle_command[1] == MODE_3) {  // 跳跃相关
        if (handle_command[0] == TROT_FORWARD_CMD || handle_command[2] - 125 < 0) {  // 前进
          trot_param.length = 0.3;
          trot_param.height = 0.03;
          trot_param.Kp = 100;
          trot_param.Kd = 4;
          TaskHandle = TrotForwardHandle;
          xTaskNotify((TaskHandle_t)TrotForwardHandle, START_ACTION, eSetValueWithoutOverwrite);
        }
        else if (handle_command[0] == TROT_BACK_CMD || handle_command[2] - 125 > 0) {  // 后退
          trot_param.length = 0.3;
          trot_param.height = 0.03;
          trot_param.Kp = 100;
          trot_param.Kd = 4;
          TaskHandle = TrotBackHandle;
          xTaskNotify((TaskHandle_t)TrotBackHandle, START_ACTION, eSetValueWithOverwrite);
        }
        else if (handle_command[0] == ROTATE_LEFT_CMD) {  // 左旋�?
          rotate_param.length = 0.2;
          rotate_param.height = 0.03;
          rotate_param.Kp = 100;
          rotate_param.Kd = 4;
          TaskHandle = RotateLeftHandle;
          xTaskNotify((TaskHandle_t)RotateLeftHandle, START_ACTION, eSetValueWithOverwrite);
        }
        else if (handle_command[0] == ROTATE_RIGHT_CMD) {  // 右旋�?
          rotate_param.length = 0.2;
          rotate_param.height = 0.03;
          rotate_param.Kp = 100;
          rotate_param.Kd = 4;
          TaskHandle = RotateRightHandle;
          xTaskNotify((TaskHandle_t)RotateRightHandle, START_ACTION, eSetValueWithOverwrite);
        }
        else if (handle_command[0] == STOP_CMD) {  // 停止
          if (TaskHandle != NULL) {
            xTaskNotify((TaskHandle_t)TaskHandle, END_ACTION, eSetValueWithOverwrite);
          }
        }
      }
      // else if (handle_command[1] == MODE_4) {
      //   if (handle_command[0] == TROT_FORWARD_CMD || handle_command[2] - 125 < 0) {
      //     TaskHandle = TrotForwardHandle;
      //     xTaskNotify((TaskHandle_t)TrotForwardHandle, START_ACTION, eSetValueWithoutOverwrite);
      //   }
      //   else if (handle_command[0] == TROT_BACK_CMD || handle_command[2] - 125 > 0) {
      //     TaskHandle = TrotBackHandle;
      //     xTaskNotify((TaskHandle_t)TrotBackHandle, START_ACTION, eSetValueWithOverwrite);
      //   }
      //   else if (handle_command[0] == ROTATE_LEFT_CMD) {  // 左旋�?
      //     TaskHandle = RotateLeftHandle;
      //     xTaskNotify((TaskHandle_t)RotateLeftHandle, START_ACTION, eSetValueWithOverwrite);
      //   }
      //   else if (handle_command[0] == ROTATE_RIGHT_CMD) {  // 右旋�?
      //     TaskHandle = RotateRightHandle;
      //     xTaskNotify((TaskHandle_t)RotateRightHandle, START_ACTION, eSetValueWithOverwrite);
      //   }
      //   else if (handle_command[0] == STOP_CMD) {  // 停止
      //     if (TaskHandle != NULL) {
      //       xTaskNotify((TaskHandle_t)TaskHandle, END_ACTION, eSetValueWithOverwrite);
      //     }
      //   }
      // }
      else if (handle_command[1] == MODE_5) {
        if (handle_command[0] == TROT_FORWARD_CMD || handle_command[2] - 125 < 0) {
          trot_param.length = 0.3;
          trot_param.height = 0.03;
          trot_param.Kp = 100;
          trot_param.Kd = 4;
          TaskHandle = TrotForwardHandle;
          xTaskNotify((TaskHandle_t)TrotForwardHandle, START_ACTION, eSetValueWithoutOverwrite);
        }
        else if (handle_command[0] == TROT_BACK_CMD || handle_command[2] - 125 > 0) {
          trot_param.length = 0.3;
          trot_param.height = 0.03;
          trot_param.Kp = 100;
          trot_param.Kd = 4;
          TaskHandle = TrotBackHandle;
          xTaskNotify((TaskHandle_t)TrotBackHandle, START_ACTION, eSetValueWithOverwrite);
        }
        else if (handle_command[0] == ROTATE_LEFT_CMD) {  // 左旋�?
          rotate_param.length = 0.2;
          rotate_param.height = 0.03;
          rotate_param.Kp = 100;
          rotate_param.Kd = 4;
          TaskHandle = RotateLeftHandle;
          xTaskNotify((TaskHandle_t)RotateLeftHandle, START_ACTION, eSetValueWithOverwrite);
        }
        else if (handle_command[0] == ROTATE_RIGHT_CMD) {  // 右旋�?
          rotate_param.length = 0.2;
          rotate_param.height = 0.03;
          rotate_param.Kp = 100;
          rotate_param.Kd = 4;
          TaskHandle = RotateRightHandle;
          xTaskNotify((TaskHandle_t)RotateRightHandle, START_ACTION, eSetValueWithOverwrite);
        }
        else if (handle_command[0] == JUMP_UP_CMD) {
          trot_param.length = 0.1;
          trot_param.height = 0.03;
          trot_param.Kp = 35;
          trot_param.Kd = 4;
          TaskHandle = TrotForwardHandle;
          xTaskNotify((TaskHandle_t)TrotForwardHandle, START_ACTION, eSetValueWithoutOverwrite);
        }
        else if (handle_command[0] == JUMP_FORWARD_CMD) {
          trot_param.length = 0.1;
          trot_param.height = 0.03;
          trot_param.Kp = 35;
          trot_param.Kd = 4;
          TaskHandle = TrotBackHandle;
          xTaskNotify((TaskHandle_t)TrotBackHandle, START_ACTION, eSetValueWithOverwrite);
        }
        else if (handle_command[0] == MOVE_LEFT_CMD) {
          rotate_param.length = 0.1;
          rotate_param.height = 0.03;
          rotate_param.Kp = 35;
          rotate_param.Kd = 4;
          TaskHandle = RotateLeftHandle;
          xTaskNotify((TaskHandle_t)RotateLeftHandle, START_ACTION, eSetValueWithOverwrite);
        }
        else if (handle_command[0] == MOVE_RIGHT_CMD) {
          rotate_param.length = 0.1;
          rotate_param.height = 0.03;
          rotate_param.Kp = 35;
          rotate_param.Kd = 4;
          TaskHandle = RotateRightHandle;
          xTaskNotify((TaskHandle_t)RotateRightHandle, START_ACTION, eSetValueWithOverwrite);
        }
        else if (handle_command[0] == STOP_CMD) {  // 停止
          if (TaskHandle != NULL) {
            xTaskNotify((TaskHandle_t)TaskHandle, END_ACTION, eSetValueWithOverwrite);
          }
        }
      }

      pre_t = t;
    }


    if (CompareCommand(last_handle_command, handle_command) != 0) {
      if (handle_command[1] == MODE_2) {  // 斜坡切换
        if (handle_command[0] == JUMP_UP_CMD) {
          isSlope = NO_SLOPE;
        }
        else if (handle_command[0] == MOVE_LEFT_CMD) {
          isSlope = SLOPE;
        }
        else if (handle_command[0] == MOVE_RIGHT_CMD) {
          isSlope = SLOPE_LR;
        }
      }
      else if (handle_command[1] == MODE_3) {  // 跳跃相关
        if (handle_command[0] == JUMP_FORWARD_CMD) {
          // 跳高�?
          is_jump_stair = 0;
          squat_l0 = jump_wall_param.squat_l0;
          squat_l1 = jump_wall_param.squat_l1;
          jump_l0 = jump_wall_param.jump_l0;
          jump_l1 = jump_wall_param.jump_l1;
          tilt_l0 = jump_wall_param.tilt_l0;
          tilt_l1 = jump_wall_param.tilt_l1;
          jump_Kp = jump_wall_param.Kp;
          jump_Kd = jump_wall_param.Kd;
          TaskHandle = JumpForwardHandle;
          xTaskNotify((TaskHandle_t)JumpForwardHandle, START_ACTION, eSetValueWithOverwrite);
        }
        else if (handle_command[0] == MOVE_LEFT_CMD) {
          // 跳阶�?
          is_jump_stair = 1;
          squat_l0 = jump_stair_param.squat_l0;
          squat_l1 = jump_stair_param.squat_l1;
          jump_l0 = jump_stair_param.jump_l0;
          jump_l1 = jump_stair_param.jump_l1;
          tilt_l0 = jump_stair_param.tilt_l0;
          tilt_l1 = jump_stair_param.tilt_l1;
          jump_Kp = jump_stair_param.Kp;
          jump_Kd = jump_stair_param.Kd;
          TaskHandle = JumpForwardHandle;
          xTaskNotify((TaskHandle_t)JumpForwardHandle, START_ACTION, eSetValueWithOverwrite);
        }
        else if (handle_command[0] == MOVE_RIGHT_CMD) {
          // 跳断�?
          is_jump_stair = 0;
          squat_l0 = jump_breakbrigde_param.squat_l0;
          squat_l1 = jump_breakbrigde_param.squat_l1;
          jump_l0 = jump_breakbrigde_param.jump_l0;
          jump_l1 = jump_breakbrigde_param.jump_l1;
          tilt_l0 = jump_breakbrigde_param.tilt_l0;
          tilt_l1 = jump_breakbrigde_param.tilt_l1;
          jump_Kp = jump_breakbrigde_param.Kp;
          jump_Kd = jump_breakbrigde_param.Kd;
          TaskHandle = JumpForwardHandle;
          xTaskNotify((TaskHandle_t)JumpForwardHandle, START_ACTION, eSetValueWithOverwrite);
        }
        else if (handle_command[0] == JUMP_UP_CMD) {
          // 第一次跳阶梯
          is_jump_stair = 1;
          squat_l0 = jump_first_stair_param.squat_l0;
          squat_l1 = jump_first_stair_param.squat_l1;
          jump_l0 = jump_first_stair_param.jump_l0;
          jump_l1 = jump_first_stair_param.jump_l1;
          tilt_l0 = jump_first_stair_param.tilt_l0;
          tilt_l1 = jump_first_stair_param.tilt_l1;
          jump_Kp = jump_first_stair_param.Kp;
          jump_Kd = jump_first_stair_param.Kd;
          TaskHandle = JumpForwardHandle;
          xTaskNotify((TaskHandle_t)JumpForwardHandle, START_ACTION, eSetValueWithOverwrite);
        }
      }
      else if (handle_command[1] == MODE_4) {
        if (handle_command[0] == TROT_FORWARD_CMD) {
          trot_param.length += 0.05;
        }
        else if (handle_command[0] == TROT_BACK_CMD) {
          trot_param.length -= 0.05;
        }
        else if (handle_command[0] == ROTATE_LEFT_CMD) {
          trot_param.height += 0.01;
        }
        else if (handle_command[0] == ROTATE_RIGHT_CMD) {
          trot_param.height -= 0.01;
        }
        else if (handle_command[0] == JUMP_UP_CMD) {
          rotate_param.length += 0.05;
        }
        else if (handle_command[0] == JUMP_FORWARD_CMD) {
          rotate_param.length -= 0.05;
        }
        else if (handle_command[0] == MOVE_LEFT_CMD) {
          rotate_param.height += 0.01;
        }
        else if (handle_command[0] == MOVE_RIGHT_CMD) {
          rotate_param.height -= 0.01;
        }
      }
      // else if (handle_command[1] == MODE_5) {

      // }
      // else if (handle_command[0] == MODE_1) {

      // }

      if (handle_command[0] == STOP_CMD && ((handle_command[1] & 0xF0) == BECOME_HIGHER)) {
        robot_height += 0.01;
        TaskHandle = NULL;
      }
      else if (handle_command[0] == STOP_CMD && ((handle_command[1] & 0xF0) == BECOME_LOWWER)) {
        robot_height -= 0.01;
        TaskHandle = NULL;
      }
    }

    memcpy(last_handle_command, handle_command, sizeof(handle_command));

    osDelay(1);
  }
  /* USER CODE END NotifyActionTask */
}

/* USER CODE BEGIN Header_ParseCameraTask */
/**
* @brief Function implementing the ParseCamera thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ParseCameraTask */
void ParseCameraTask(void *argument)
{
  /* USER CODE BEGIN ParseCameraTask */

  uint8_t rx_camera[RX_BTYES_LENGTH];

  /* Infinite loop */
  for(;;)
  {
    // if (xQueueReceive(camera_queue, rx_camera, portMAX_DELAY) == pdPASS) {
    //   if (check_rx_bytes(rx_camera) == RX_NORMAL) {
    //     parse_camera_bytes(rx_camera);
    //   }
    // }
    if (check_rx_bytes(rx_bytes)) {
      parse_camera_bytes(rx_bytes);
    }
    osDelay(1);
  }
  /* USER CODE END ParseCameraTask */
}

/* USER CODE BEGIN Header_TurnLeftTask */
/**
* @brief Function implementing the TurnLeft thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TurnLeftTask */
void TurnLeftTask(void *argument)
{
  /* USER CODE BEGIN TurnLeftTask */

  uint32_t notify_value = 0;

  /* Infinite loop */
  for(;;)
  {
    if (xTaskNotifyWait(0, 0, &notify_value, 0) == pdTRUE) {
      if (notify_value == START_ACTION) {
        if (turn_controller.turn_state == EndTurn) {
          turn_controller.turn_state = PreTurn;
          turn_controller.turn_angular_direction = TurnLeft;
          t = 0;
        }
      }
      else if (notify_value == END_ACTION) {
        if (turn_controller.turn_state == Turning) {
          isStop = NEED_TO_STOP;
        }
      }

      if (turn_controller.turn_state != EndTurn) {
        Turn_FSM(&turn_controller, 0.04, 0.1, 0.03, robot_height);

        if (turn_controller.turn_state != EndTurn) {
          __HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);
          __HAL_TIM_SET_COUNTER(&htim2, 0);
          HAL_TIM_Base_Start_IT(&htim2);
        }
      }

      notify_value = 0;
    }

    osDelay(1);
  }
  /* USER CODE END TurnLeftTask */
}

/* USER CODE BEGIN Header_TurnRightTask */
/**
* @brief Function implementing the TurnRight thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TurnRightTask */
void TurnRightTask(void *argument)
{
  /* USER CODE BEGIN TurnRightTask */

  uint32_t notify_value = 0;

  /* Infinite loop */
  for(;;)
  {
    if (xTaskNotifyWait(0, 0, &notify_value, 0) == pdTRUE) {
      if (notify_value == START_ACTION) {
        if (turn_controller.turn_state == EndTurn) {
          turn_controller.turn_state = PreTurn;
          turn_controller.turn_angular_direction = TurnRight;
          t = 0;
        }
      }
      else if (notify_value == END_ACTION) {
        if (turn_controller.turn_state == Turning) {
          isStop = NEED_TO_STOP;
        }
      }

      if (turn_controller.turn_state != EndTurn) {
        Turn_FSM(&turn_controller, 0.04, 0.1, 0.03, robot_height);

        if (turn_controller.turn_state != EndTurn) {
          __HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);
          __HAL_TIM_SET_COUNTER(&htim2, 0);
          HAL_TIM_Base_Start_IT(&htim2);
        }
      }

      notify_value = 0;
    }

    osDelay(1);
  }
  /* USER CODE END TurnRightTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */


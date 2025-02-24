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
#include "SBUS.h"
#include "board_comm.h"
#include "Task_manager.h"
#include "bsp_dwt.h"
#include "BSP_fdcan.h"
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
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for SBUSTask */
osThreadId_t SBUSTaskHandle;
const osThreadAttr_t SBUSTask_attributes = {
  .name = "SBUSTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for BoardcommTask */
osThreadId_t BoardcommTaskHandle;
const osThreadAttr_t BoardcommTask_attributes = {
  .name = "BoardcommTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartSBUSTask(void *argument);
void StartBoardcommTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

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

  /* creation of SBUSTask */
  SBUSTaskHandle = osThreadNew(StartSBUSTask, NULL, &SBUSTask_attributes);

  /* creation of BoardcommTask */
  BoardcommTaskHandle = osThreadNew(StartBoardcommTask, NULL, &BoardcommTask_attributes);

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

/* USER CODE BEGIN Header_StartSBUSTask */
/**
* @brief Function implementing the SBUSTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSBUSTask */
void StartSBUSTask(void *argument)
{
  /* USER CODE BEGIN StartSBUSTask */
		static float sbus_start;
  static float sbus_dt;
		SBUS_Init();
  /* Infinite loop */
  for(;;)
  {
		sbus_start= DWT_GetTimeline_ms();
		SBUS_Task();
		sbus_dt = DWT_GetTimeline_ms() - sbus_start;
    osDelay(1);
  }
  /* USER CODE END StartSBUSTask */
}

/* USER CODE BEGIN Header_StartBoardcommTask */
/**
* @brief Function implementing the BoardcommTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartBoardcommTask */
void StartBoardcommTask(void *argument)
{
  /* USER CODE BEGIN StartBoardcommTask */

	static float boardcomm_start;
  static float boardcomm_dt;
	BoardCommInit(&board_comm,&hfdcan2, 2);
  /* Infinite loop */
  for(;;)
  {
		boardcomm_start = DWT_GetTimeline_ms();
		boardCommunicateTask();
		boardcomm_dt = DWT_GetTimeline_ms() - boardcomm_start;
  }
  /* USER CODE END StartBoardcommTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */


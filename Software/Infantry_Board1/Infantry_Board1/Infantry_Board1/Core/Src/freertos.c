/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "pid.h"
#include "BSP_Fdcan.h"
#include "BSP_Usart.h"

#include "Cloud_Control.h"
#include "Shoot.h"

#include "Task_LED.h"
#include "SBUS.h"
#include "DT7.h"
#include "BSP_Test.h"
#include "Saber_C3.h"
#include "Protocol_UpperComputer.h"

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
/***********Queues************/
QueueHandle_t FDCAN1_ReceiveHandle;
QueueHandle_t FDCAN2_ReceiveHandle;//ï¿½ï¿½Ê±ï¿½ò¿ª£ï¿½ï¿½ï¿½È»Ã»ï¿½ï¿½
QueueHandle_t FDCAN_SendHandle;
QueueHandle_t Communicate_ReceivefromPCHandle;
/***********Tasks*************/
osThreadId Task_Fdcan1MsgRecHandle;
osThreadId Task_Fdcan2MsgRecHandle;//ï¿½ï¿½Ê±ï¿½ò¿ª£ï¿½ï¿½ï¿½È»Ã»ï¿½ï¿½
osThreadId Task_FdcanSendHandle;
osThreadId Move_DataHandle;
osThreadId led_RGB_flow_handle;
osThreadId Robot_Control_Handle;
osThreadId Task_CommunicateFromPC_Handle;
osThreadId Task_CommunicateToPC_Handle;
osThreadId Task_OffLineCheck_Handle;

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId StartTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
extern void Fdcan1Receives(void const *argument);
extern void Fdcan2Receives(void const *argument);
extern void AllFdcanSend(void const *argument);
extern void Robot_Control(void const *argument);
extern void Robot_Control(void const *argument);
extern void USBCommunicateTask_Receive(void const *argument);
extern void USBCommunicateTask_Send(void const *argument);
extern void Off_Line_Check(void const *argument);

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void ALL_Init(void const * argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

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
	
  /* definition and creation of CAN1_Receive */
  FDCAN1_ReceiveHandle = xQueueCreate(16, sizeof(Fdcan_Export_Data_t));

  /* definition and creation of CAN2_Receive */
  FDCAN2_ReceiveHandle = xQueueCreate(16, sizeof(Fdcan_Export_Data_t));

  /* definition and creation of CAN_Send */
  FDCAN_SendHandle = xQueueCreate(16, sizeof(Fdcan_Send_Data_t));

  /* definition and creation of Communicate_PC */
  Communicate_ReceivefromPCHandle = xQueueCreate(16, UpperCom_MAX_BUF);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of StartTask */
  osThreadDef(StartTask, ALL_Init, osPriorityRealtime, 0, 128);
  StartTaskHandle = osThreadCreate(osThread(StartTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
	
  /* definition and creation of Can1ReceiveTask */
  osThreadDef(Fdcan1_ReceiveTask, Fdcan1Receives, osPriorityAboveNormal, 0, 256);
  Task_Fdcan1MsgRecHandle = osThreadCreate(osThread(Fdcan1_ReceiveTask), NULL);

  /* definition and creation of Can1ReceiveTask */
  osThreadDef(Fdcan2_ReceiveTask, Fdcan2Receives, osPriorityAboveNormal, 0, 256);
  Task_Fdcan2MsgRecHandle = osThreadCreate(osThread(Fdcan2_ReceiveTask), NULL);

  /* definition and creation of CanSendTask */
  osThreadDef(Fdcan_SendTask, AllFdcanSend, osPriorityHigh, 0, 256);
  Task_FdcanSendHandle = osThreadCreate(osThread(Fdcan_SendTask), NULL);

  /* definition and creation of LED_RGB_Flow_Task */
  osThreadDef(led, led_RGB_flow_task, osPriorityNormal, 0, 128);
  led_RGB_flow_handle = osThreadCreate(osThread(led), NULL);

  /* definition and creation of Robot_Control_Task */
  osThreadDef(Robot_Control_Task, Robot_Control, osPriorityHigh, 0, 512);
  Robot_Control_Handle = osThreadCreate(osThread(Robot_Control_Task), NULL);

  /* definition and creation of Task_CommunicateToPC_Handle */
  osThreadDef(Task_CommunicateToPC_Handle, USBCommunicateTask_Send, osPriorityAboveNormal, 0, 256);
  Task_CommunicateToPC_Handle = osThreadCreate(osThread(Task_CommunicateToPC_Handle), NULL);
		
  /* definition and creation of Task_OffLineCheck_Handle */
  osThreadDef(Task_OffLineCheck_Handle, Off_Line_Check, osPriorityHigh, 0, 256);
  Task_OffLineCheck_Handle = osThreadCreate(osThread(Task_OffLineCheck_Handle), NULL);

  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_ALL_Init */
/**
* @brief Function implementing the StartTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ALL_Init */
void ALL_Init(void const * argument)
{
  /* USER CODE BEGIN ALL_Init */
    /* Infinite loop */
    for (;;)
    {
        taskENTER_CRITICAL();                 //ï¿½ï¿½ï¿½ï¿½ï¿½Ù½ï¿½ï¿½ï¿½

        /* FDCANï¿½Ð¶Ï³ï¿½Ê¼ï¿½ï¿½ */
        Fdcan_Fun.FDCAN_IT_Init(&hfdcan1, Fdcan1_Type);
        Fdcan_Fun.FDCAN_IT_Init(&hfdcan2, Fdcan2_Type);
			  /*Ä£ºýPID³õÊ¼»¯*/
				fuzzy_init(&fuzzy_pid_shoot_l,30,-30,4,0.01,3);
				fuzzy_init(&fuzzy_pid_shoot_r,30,-30,4,0.01,3);
				fuzzy_init(&fuzzy_pid_bullet_v,30,-30,5,0.01,3);	
				fuzzy_init(&fuzzy_pid_bullet_l,10,-10,0.1,0.01,1);	
//			  fuzzy_init(&fuzzy_pid_pitch_in,10,-10,0.1,0.01,1);	
//				fuzzy_init(&fuzzy_pid_pitch_out,0.025,-0.00005,0.1,0.01,1);	
			
//			  /*Éñ¾­ÍøÂçPID³õÊ¼»¯*/
//				init_random();
//				init_neural_network();
        /*PID³õÊ¼»¯*/
        Incremental_PIDInit(&M3508_FricL_Pid, 30.0f, 0.5f, 24, 10000, 1000);//×óÄ¦²ÁÂÖËÙ¶È»·
        Incremental_PIDInit(&M3508_FricR_Pid, 25.f, 0.5f, 22, 10000, 1000);//ÓÒÄ¦²ÁÂÖËÙ¶È»·
        Position_PIDInit(&M2006_DialV_Pid, 0.5f, 0.001f, 0.5, 0.5, 20000, 8000, 700);//²¦ÅÌµç»úËÙ¶È»·
        Incremental_PIDInit(&M2006_DialI_Pid, 12.5f, 0.5f, 3, 10000, 1000);//²¦ÅÌµç»úµçÁ÷»·
        /*Pitch³õÊ¼»¯*/
        Position_PIDInit(&M6020s_PitchIPID, 150.0f, 1.5 ,80, 0, 20000, 10000, 3000);
        Position_PIDInit(&M6020s_PitchOPID, 4.f, 0.005, 1.1f, 0.1f, 20000, 10000, 3000);
        /* ×ÔÃéPID³õÊ¼»¯ */
        Position_PIDInit(&AutoAim_M6020s_PitchIPID, 1300.0f, 0, 600, 10, 12000, 10000, 0);
        Position_PIDInit(&AutoAim_M6020s_PitchOPID, 0.046f, 0, 0.1f, 1.f, 20000, 10000, 10000);
        /*Éè±¸³õÊ¼»¯*/
        //ÔÆÌ¨³õÊ¼»¯
        Cloud_Init();
        //Ò£¿ØÆ÷³õÊ¼»¯
#if (RemoteControlMethod == TDF)
        SBUS_Init();//ï¿½ï¿½Ø·ï¿?
#elif (RemoteControlMethod == DT7)
        DT7_Init();//DT7ï¿½ï¿½
#endif
        vTaskDelete(StartTaskHandle);         //É¾ï¿½ï¿½ï¿½ï¿½Ç°ï¿½ï¿½ï¿½ï¿½
        taskEXIT_CRITICAL();                  //ï¿½Ë³ï¿½ï¿½Ù½ï¿½ï¿½ï¿½
    }
  /* USER CODE END ALL_Init */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

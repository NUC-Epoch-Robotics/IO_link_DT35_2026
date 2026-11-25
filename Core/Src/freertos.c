/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "iol_port_hal.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
/* Definitions for DT35_1_App_task */
osThreadId_t DT35_1_App_taskHandle;
const osThreadAttr_t DT35_1_App_task_attributes = {
  .name = "DT35_1_App_task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for DT35_2_App_task */
osThreadId_t DT35_2_App_taskHandle;
const osThreadAttr_t DT35_2_App_task_attributes = {
  .name = "DT35_2_App_task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for DT35_3_App_task */
osThreadId_t DT35_3_App_taskHandle;
const osThreadAttr_t DT35_3_App_task_attributes = {
  .name = "DT35_3_App_task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for DT35_4_App_task */
osThreadId_t DT35_4_App_taskHandle;
const osThreadAttr_t DT35_4_App_task_attributes = {
  .name = "DT35_4_App_task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for DT35_App_task */
osThreadId_t DT35_App_taskHandle;
const osThreadAttr_t DT35_App_task_attributes = {
  .name = "DT35_App_task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
extern void DT35_1_App_Task(void *argument);
extern void DT35_2_App_Task(void *argument);
extern void DT35_3_App_Task(void *argument);
extern void DT35_4_App_Task(void *argument);
extern void DT35_App_Task(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  IOL_HAL_RtosInit();
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

  /* creation of DT35_1_App_task */
  DT35_1_App_taskHandle = osThreadNew(DT35_1_App_Task, NULL, &DT35_1_App_task_attributes);

  /* creation of DT35_2_App_task */
  DT35_2_App_taskHandle = osThreadNew(DT35_2_App_Task, NULL, &DT35_2_App_task_attributes);

  /* creation of DT35_3_App_task */
  DT35_3_App_taskHandle = osThreadNew(DT35_3_App_Task, NULL, &DT35_3_App_task_attributes);

  /* creation of DT35_4_App_task */
  DT35_4_App_taskHandle = osThreadNew(DT35_4_App_Task, NULL, &DT35_4_App_task_attributes);

  /* creation of DT35_App_task */
  DT35_App_taskHandle = osThreadNew(DT35_App_Task, NULL, &DT35_App_task_attributes);

  if ((defaultTaskHandle == NULL) || (DT35_1_App_taskHandle == NULL) ||
      (DT35_2_App_taskHandle == NULL) || (DT35_3_App_taskHandle == NULL) ||
      (DT35_4_App_taskHandle == NULL) || (DT35_App_taskHandle == NULL)) {
    Error_Handler();
  }

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

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    (void)pcTaskName;
    taskDISABLE_INTERRUPTS();
    for (;;) { }
}

void vApplicationMallocFailedHook(void)
{
    taskDISABLE_INTERRUPTS();
    for (;;) { }
}

/* USER CODE END Application */


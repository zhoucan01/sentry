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
osThreadId defaultTaskHandle;
osThreadId system_taskHandle;
osThreadId INSTaskHandle;
osThreadId motor_taskHandle;

osThreadId gimbal_taskHandle;
osThreadId shoot_taskHandle;
osThreadId printHandle;
osThreadId trig_taskHandle;
osThreadId DETECTHandle;
/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void system_run(void const * argument);
void StartINSTask(void const * argument);
void gimbal_run(void const * argument);
void motor_run(void const * argument);
void shoot_run(void const * argument);
void trig_run(void const * argument);
	void printf_task(void const * argument);
  extern void DETECT_task(void const * argument);
//void motor_run(void const * argument);
//void Odometer_run(void const * argument);
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
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of system_task */
  osThreadDef(system_task, system_run, osPriorityNormal, 0, 256);
  system_taskHandle = osThreadCreate(osThread(system_task), NULL);

  /* definition and creation of INSTask */
  osThreadDef(INSTask, StartINSTask, osPriorityNormal, 0, 512);
  INSTaskHandle = osThreadCreate(osThread(INSTask), NULL);
  
  
  osThreadDef(DETECT, DETECT_task, osPriorityNormal, 0, 256);
  DETECTHandle = osThreadCreate(osThread(DETECT), NULL);

osThreadDef(motor_task, motor_run, osPriorityNormal, 0, 256);
  motor_taskHandle = osThreadCreate(osThread(motor_task), NULL);
  /* definition and creation of motor_task */
//  osThreadDef(motor_task, motor_run, osPriorityAboveNormal, 0, 256);
//  motor_taskHandle = osThreadCreate(osThread(motor_task), NULL);
  
  osThreadDef(gimbal_task, gimbal_run, osPriorityNormal, 0, 256);
  gimbal_taskHandle = osThreadCreate(osThread(gimbal_task), NULL);
  
  osThreadDef(shoot_task, shoot_run, osPriorityNormal, 0, 256);
  shoot_taskHandle = osThreadCreate(osThread(shoot_task), NULL);
  
  
  osThreadDef(trig_task, trig_run, osPriorityNormal, 0, 256);
  trig_taskHandle = osThreadCreate(osThread(trig_task), NULL);
  
osThreadDef(printfTask, printf_task, osPriorityLow, 0, 128);
printHandle = osThreadCreate(osThread(printfTask), NULL);
//osThreadDef(Odometer_task, Odometer_run, osPriorityAboveNormal, 0, 256);
//  Odometer_taskHandle = osThreadCreate(osThread(Odometer_task), NULL);
  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
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
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_system_run */
/**
* @brief Function implementing the system_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_system_run */
__weak void system_run(void const * argument)
{
  /* USER CODE BEGIN system_run */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END system_run */
}

/* USER CODE BEGIN Header_StartINSTask */
/**
* @brief Function implementing the INSTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartINSTask */
__weak void StartINSTask(void const * argument)
{
  /* USER CODE BEGIN StartINSTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartINSTask */
}

/* USER CODE BEGIN Header_StartTask04 */
/**
* @brief Function implementing the motor_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask04 */
__weak void StartTask04(void const * argument)
{
  /* USER CODE BEGIN StartTask04 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTask04 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */



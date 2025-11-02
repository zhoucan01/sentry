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
osThreadId chassis_taskHandle;
//osThreadId Odometer_taskHandle;
osThreadId transmit_taskHandle;
osThreadId big_yaw_taskHandle;
osThreadId small_gim_taskHandle;
osThreadId REFEREEHandle;
osThreadId AutoHandle;
/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void referee_transmit(void const * argument);
void system_run(void const * argument);
void StartINSTask(void const * argument);
void motor_run(void const * argument);
void chassis_task_run(void const * argument);
void Transmit_run(void const * argument);
void big_yaw_run(void const * argument);
void small_gimbal_run(void const * argument);
extern void REFEREE_Task(void const * argument);
void Auto_run(void const * argument);

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
  osThreadDef(defaultTask, referee_transmit, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of system_task */
  osThreadDef(system_task, system_run, osPriorityNormal, 0, 256);
  system_taskHandle = osThreadCreate(osThread(system_task), NULL);

  /* definition and creation of INSTask */
  osThreadDef(INSTask, StartINSTask, osPriorityNormal, 0, 512);
  INSTaskHandle = osThreadCreate(osThread(INSTask), NULL);



  /* definition and creation of motor_task */
  osThreadDef(motor_task, motor_run, osPriorityNormal, 0, 128);
  motor_taskHandle = osThreadCreate(osThread(motor_task), NULL);
  
  
   osThreadDef(chassis_task, chassis_task_run, osPriorityNormal, 0, 128);
  chassis_taskHandle = osThreadCreate(osThread(chassis_task), NULL);
  
   osThreadDef(transmit_task, Transmit_run, osPriorityNormal, 0, 128);
  transmit_taskHandle = osThreadCreate(osThread(transmit_task), NULL);
  
  osThreadDef(big_yaw_task, big_yaw_run, osPriorityNormal, 0, 128);
  big_yaw_taskHandle = osThreadCreate(osThread(big_yaw_task), NULL);
  
  osThreadDef(small_gim_task, small_gimbal_run, osPriorityNormal, 0, 128);
  small_gim_taskHandle = osThreadCreate(osThread(small_gim_task), NULL);
  
   osThreadDef(REFEREE, REFEREE_Task, osPriorityNormal, 0, 256);
  REFEREEHandle = osThreadCreate(osThread(REFEREE), NULL);


  osThreadDef(Auto_task, Auto_run, osPriorityNormal, 0, 512);
  AutoHandle = osThreadCreate(osThread(Auto_task), NULL);
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
//void StartDefaultTask(void const * argument)
//{
//  /* USER CODE BEGIN StartDefaultTask */
//  /* Infinite loop */
//  for(;;)
//  {
//  
//    
//    osDelay(1);
//  }
//  /* USER CODE END StartDefaultTask */
//}

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

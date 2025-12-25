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
#include "bsp_can.h"
#include "pid.h"
#include "bsp_uart.h"
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
//底盘电机变量
extern moto_info_t motor_info[MOTOR_MAX_NUM];
extern pid_struct_t motor_pid[7];
//遥控器变量
extern rc_info_t rc;

/* USER CODE END Variables */
osThreadId ChassisHandle;
osThreadId GimbalHandle;
osThreadId RemoteControlHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartChassisTask(void const * argument);
void StartGimbalTask(void const * argument);
void StartRcTask(void const * argument);

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
  /* definition and creation of Chassis */
  osThreadDef(Chassis, StartChassisTask, osPriorityAboveNormal, 0, 128);
  ChassisHandle = osThreadCreate(osThread(Chassis), NULL);

  /* definition and creation of Gimbal */
  osThreadDef(Gimbal, StartGimbalTask, osPriorityRealtime, 0, 128);
  GimbalHandle = osThreadCreate(osThread(Gimbal), NULL);

  /* definition and creation of RemoteControl */
  osThreadDef(RemoteControl, StartRcTask, osPriorityHigh, 0, 128);
  RemoteControlHandle = osThreadCreate(osThread(RemoteControl), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartChassisTask */
/**
  * @brief  Function implementing the Chassis thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartChassisTask */
void StartChassisTask(void const * argument)
{
  /* USER CODE BEGIN StartChassisTask */
  extern int16_t Vx,Vy;
  extern int16_t wheel_speed[4];

  /* Infinite loop */
  for(;;)
  {
    if (rc.sw1==3)//平移模式
    {
      Vx=rc.ch3;
      Vy=rc.ch4;

      wheel_speed[0] = -Vx - Vy ;
      wheel_speed[1] = Vx - Vy ;
      wheel_speed[2] = Vx + Vy ;
      wheel_speed[3] = -Vx + Vy;
      for (uint8_t i = 0; i < 4; i++)
      {
        motor_info[i].set_voltage = pid_calc(&motor_pid[i], wheel_speed[i], motor_info[i].rotor_speed);
      }
      /* send motor control message through can bus*/
      set_motor_voltage(hcan2,
                      0,
                      motor_info[0].set_voltage,
                      motor_info[1].set_voltage,
                      motor_info[2].set_voltage,
                      motor_info[3].set_voltage);
    }
    else if (rc.sw1==1)//旋转模式
    {
      Vx=rc.ch3;
      Vy=rc.ch4;

      wheel_speed[0] = -Vx - Vy ;
      wheel_speed[1] = Vx - Vy ;
      wheel_speed[2] = Vx + Vy ;
      wheel_speed[3] = -Vx + Vy;
      for (uint8_t i = 0; i < 4; i++)
      {
        motor_info[i].set_voltage = pid_calc(&motor_pid[i], wheel_speed[i], motor_info[i].rotor_speed);
      }
      /* send motor control message through can bus*/
      set_motor_voltage(hcan2,
                      0,
                      motor_info[0].set_voltage,
                      motor_info[1].set_voltage,
                      motor_info[2].set_voltage,
                      motor_info[3].set_voltage);
    }
    osDelay(1);
  }
  /* USER CODE END StartChassisTask */
}

/* USER CODE BEGIN Header_StartGimbalTask */
/**
* @brief Function implementing the Gimbal thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartGimbalTask */
void StartGimbalTask(void const * argument)
{
  /* USER CODE BEGIN StartGimbalTask */

  for(;;)
  {

    osDelay(1);
  }
  /* USER CODE END StartGimbalTask */
}

/* USER CODE BEGIN Header_StartRcTask */
/**
* @brief Function implementing the RemoteControl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRcTask */
void StartRcTask(void const * argument)
{
  /* USER CODE BEGIN StartRcTask */

  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartRcTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

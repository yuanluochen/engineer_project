/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       start_task.c/h
  * @brief      启动任务，将一个个任务开启，分配资源，给定任务优先级,
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */

#include "Start_Task.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "Detect_Task.h"
#include "Calibrate_Task.h"
#include "User_Task.h"
#include "INS_Task.h"
#include "Chassis_Task.h"
#include "Gimbal_Task.h"
#include "Robot_arm_task.h"
#include "servo_task.h"

#define INS_TASK_PRIO 20
#define INS_TASK_SIZE 512
static TaskHandle_t INSTask_Handler;

#if ENGINEER
#define GIMBAL_TASK_PRIO 19
#define GIMBAL_STK_SIZE 512
TaskHandle_t GIMBALTask_Handler;
#endif

#define Chassis_TASK_PRIO 18
#define Chassis_STK_SIZE 512
TaskHandle_t ChassisTask_Handler;

#define User_TASK_PRIO 4
#define User_STK_SIZE 256
static TaskHandle_t UserTask_Handler;

#define START_TASK_PRIO 1
#define START_STK_SIZE 512
static TaskHandle_t StartTask_Handler;

#if ENGINEER
#define CALIBRATE_TASK_PRIO 5
#define CALIBRATE_STK_SIZE 512
static TaskHandle_t CalibrateTask_Handler;
#endif

#define Detect_TASK_PRIO 10
#define Detect_STK_SIZE 512
static TaskHandle_t DetectTask_Handler;

#if ENGINEER_ARM
#define ROBOT_ARM_TASK_PRIO 19
#define ROBOT_ARM_SIZE 512
static TaskHandle_t RobotArmTask_Handler;
#endif

#if ENGINEER_SERVO
#define SERVO_SIZE 512
#define SERVO_TASK_PRIO 15
static TaskHandle_t ServoTask_Handler;
#endif

void start_task(void *pvParameters)
{
    taskENTER_CRITICAL();

    xTaskCreate((TaskFunction_t)INSTask,
                (const char *)"INSTask",
                (uint16_t)INS_TASK_SIZE,
                (void *)NULL,
                (UBaseType_t)INS_TASK_PRIO,
                (TaskHandle_t *)&INSTask_Handler);

#if ENGINEER
    xTaskCreate((TaskFunction_t)GIMBAL_task,
                (const char *)"GIMBAL_task",
                (uint16_t)GIMBAL_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)GIMBAL_TASK_PRIO,
                (TaskHandle_t *)&GIMBALTask_Handler);
#endif

    xTaskCreate((TaskFunction_t)chassis_task,
                (const char *)"ChassisTask",
                (uint16_t)Chassis_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)Chassis_TASK_PRIO,
                (TaskHandle_t *)&ChassisTask_Handler);

    xTaskCreate((TaskFunction_t)UserTask,
                (const char *)"UserTask",
                (uint16_t)User_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)User_TASK_PRIO,
                (TaskHandle_t *)&UserTask_Handler);

#if ENGINEER
    xTaskCreate((TaskFunction_t)calibrate_task,
                (const char *)"CaliTask",
                (uint16_t)CALIBRATE_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)CALIBRATE_TASK_PRIO,
                (TaskHandle_t *)&CalibrateTask_Handler);
#endif

    xTaskCreate((TaskFunction_t)DetectTask,
                (const char *)"DetectTask",
                (uint16_t)Detect_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)Detect_TASK_PRIO,
                (TaskHandle_t *)&DetectTask_Handler);
    
#if ENGINEER_ARM

    xTaskCreate((TaskFunction_t)Robot_arm_task,
                (const char*)"Robot_arm_task",
                (uint16_t)ROBOT_ARM_SIZE,
                (void *)NULL,
                (UBaseType_t)ROBOT_ARM_TASK_PRIO,
                (TaskHandle_t *)&RobotArmTask_Handler);

#endif
#if ENGINEER_SERVO
    xTaskCreate((TaskFunction_t)servo_task,
                (const char*)"servo_task",
                (uint16_t)SERVO_SIZE,
                (void*)NULL,
                (UBaseType_t)SERVO_TASK_PRIO,
                (TaskHandle_t *)&ServoTask_Handler);
#endif

    vTaskDelete(StartTask_Handler); //删除开始任务
    taskEXIT_CRITICAL();            //退出临界区
}

void startTast(void)
{
    xTaskCreate((TaskFunction_t)start_task,          //任务函数
                (const char *)"start_task",          //任务名称
                (uint16_t)START_STK_SIZE,            //任务堆栈大小
                (void *)NULL,                        //传递给任务函数的参数
                (UBaseType_t)START_TASK_PRIO,        //任务优先级
                (TaskHandle_t *)&StartTask_Handler); //任务句柄
}

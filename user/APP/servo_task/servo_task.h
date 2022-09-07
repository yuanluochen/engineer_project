#ifndef SERVO_TASK_H
#define SERVO_TASK_H

#include "stm32f4xx.h"
#include "main.h"
#include "user_lib.h"
#include "chassis_task.h"

#define SERVO_TASK_INIT_TIME 357

//一阶滤波控制时间
#define SERVO_CONTROL_TIME 0.002f

#define SERVO_ACCEL_ANGLE_NUM CHASSIS_ACCEL_X_NUM

//遥控器死区
#define SERVO_RC_DEADLINE 10

//舵机控制遥控器
#define SERVO_CONTROL_CHANNEL 2
//遥控器与舵机转换比例
#define SERVO_ANGLE_RC_SEN 0.1068883610451f
//舵机角度数值转化为定时器自动重装载值
#define SERVO_ANGLE_TO_ARR 0.1388888888888f

#define SERVO_CONTROL_TIME_MS 2



//自动重装载器，该位控制Servo转动角度
//数值为5，舵机转动角度为0度
//数值为10，舵机转动角度为45度
//      15              90
//      20              135
//      25              180
typedef enum
{
    SERVO_ANGLE_0   = 5,
    SERVO_ANGLE_45  = 10,
    SERVO_ANGLE_90  = 15,
    SERVO_ANGLE_135 = 20,
    SERVO_ANGLE_180 = 25,
}Servo_angle;

//舵机状态枚举
typedef enum
{
    SERVO_STATE_ENABLE,//舵机使能
    SERVO_STATE_DISABLE,//舵机无力
}Servo_mode_t;

typedef struct
{
    const RC_ctrl_t* servo_RC;//遥控器指针

    Servo_mode_t servo_mode;//舵机模式
    Servo_mode_t last_servo_mode;//舵机上次的模式

    first_order_filter_type_t servo_cmd_slow_set_angle; 

    fp32 angle_max;//角度最大值
    fp32 angle_min;//角度最小值

    fp32 angle_set;//角度设置

}servo_move_t;



void servo_task( void * pvParameters);
#endif

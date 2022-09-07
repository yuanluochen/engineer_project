/**
 * @file Robot_arm_task.h
 * @author yuanluochen(you@domain.com)
 * @brief 工程手臂电机控制
 * @version 0.1
 * @date 2022-09-02
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef ROBOT_ARM_TASK
#define ROBOT_ARM_TASK

#include "remote_control.h"
#include "CAN_receive.h"
#include "user_lib.h"

//手臂电机角度环PID
#define ARM_PID_ANGLE_KP 120.0f
#define ARM_PID_ANGLE_KI 0.1f
#define ARM_PID_ANGLE_KD 500.0f
#define ARM_PID_ANGLE_N 0.0f
//角度限值
#define ARM_PID_ANGLE_MAX 180.0f
#define ARM_PID_ANGLE_MIN 0.0f
//手臂电机速度环PID
#define ARM_PID_SPEED_KP 120.0f
#define ARM_PID_SPEED_KI 0.1f
#define ARM_PID_SPEED_KD 500.0f
#define ARM_PID_SPEED_N 0.0f
//速度限制
#define ARM_PID_SPEED_MAX 4.0f

//遥控器模式切换通道值S2,up 手臂电机驱动
#define MODE_ARM 1

//该任务控制电机数量
#define ARM_MOTOR_NUM 2

#define ARM_CONTROL_TIME 0.002

#define ARM_ACCEL_ANGLE_NUM 0.1666666667f
//手臂电机最大运动角度
#define ARM_MOTOR_MAX_ANGLE 45

//手臂电机遥控器通道3
#define ARM_CONTROL_CHANNEL 2
//遥控器死区
#define ARM_RC_DEADLINE 20
//遥控器前进摇杆转化为手臂电机角速度(max 570 即最大速度为 30rad/s)
#define ARM_ANGLE_RC_SEN 0.0174f

//机器人手臂状态
typedef enum
{
    ARM_VECTOR_RAW,//手臂电机无力状态
    ARM_VECTOR_YAW,//手臂电机使能状态
}arm_move_e;

typedef struct 
{
    //手臂电机数据结构体
    const motor_measure_t *arm_motor_measure;

    //手臂电机电流
    uint16_t give_current;
    
}arm_motor_t;

//机器人手臂运动结构体
typedef struct
{
    //底盘上次移动模式
    arm_move_e arm_last_mode;
    //手臂电机移动模式
    arm_move_e arm_mode;
    //遥控器指针
    const RC_ctrl_t* arm_RC;

    //手臂电机状态数据
    arm_motor_t motor_arm[ARM_MOTOR_NUM];

    //手臂电机速度
    fp32 speed;
    //手臂电机运动角度
    fp32 angle;

    //手臂电机运动设定角度
    fp32 angle_set;

    first_order_filter_type_t Robot_arm_cmd_slow_set_angle;

}arm_move_t;

#define rc_deadline_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }
void Robot_arm_task( void* Parameters);

#endif

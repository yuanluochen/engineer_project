/**
 * @file Robot_arm_task.h
 * @author yuanluochen(you@domain.com)
 * @brief �����ֱ۵������
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

//�ֱ۵���ǶȻ�PID
#define ARM_PID_ANGLE_KP 120.0f
#define ARM_PID_ANGLE_KI 0.1f
#define ARM_PID_ANGLE_KD 500.0f
#define ARM_PID_ANGLE_N 0.0f
//�Ƕ���ֵ
#define ARM_PID_ANGLE_MAX 180.0f
#define ARM_PID_ANGLE_MIN 0.0f
//�ֱ۵���ٶȻ�PID
#define ARM_PID_SPEED_KP 120.0f
#define ARM_PID_SPEED_KI 0.1f
#define ARM_PID_SPEED_KD 500.0f
#define ARM_PID_SPEED_N 0.0f
//�ٶ�����
#define ARM_PID_SPEED_MAX 4.0f

//ң����ģʽ�л�ͨ��ֵS2,up �ֱ۵������
#define MODE_ARM 1

//��������Ƶ������
#define ARM_MOTOR_NUM 2

#define ARM_CONTROL_TIME 0.002

#define ARM_ACCEL_ANGLE_NUM 0.1666666667f
//�ֱ۵������˶��Ƕ�
#define ARM_MOTOR_MAX_ANGLE 45

//�ֱ۵��ң����ͨ��3
#define ARM_CONTROL_CHANNEL 2
//ң��������
#define ARM_RC_DEADLINE 20
//ң����ǰ��ҡ��ת��Ϊ�ֱ۵�����ٶ�(max 570 ������ٶ�Ϊ 30rad/s)
#define ARM_ANGLE_RC_SEN 0.0174f

//�������ֱ�״̬
typedef enum
{
    ARM_VECTOR_RAW,//�ֱ۵������״̬
    ARM_VECTOR_YAW,//�ֱ۵��ʹ��״̬
}arm_move_e;

typedef struct 
{
    //�ֱ۵�����ݽṹ��
    const motor_measure_t *arm_motor_measure;

    //�ֱ۵������
    uint16_t give_current;
    
}arm_motor_t;

//�������ֱ��˶��ṹ��
typedef struct
{
    //�����ϴ��ƶ�ģʽ
    arm_move_e arm_last_mode;
    //�ֱ۵���ƶ�ģʽ
    arm_move_e arm_mode;
    //ң����ָ��
    const RC_ctrl_t* arm_RC;

    //�ֱ۵��״̬����
    arm_motor_t motor_arm[ARM_MOTOR_NUM];

    //�ֱ۵���ٶ�
    fp32 speed;
    //�ֱ۵���˶��Ƕ�
    fp32 angle;

    //�ֱ۵���˶��趨�Ƕ�
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

/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       can_receive.c/h
  * @brief      ���can�豸�����շ����������ļ���ͨ��can�ж���ɽ���?
  * @note       ���ļ�����freeRTOS����
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. ���?
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */

#ifndef CANTASK_H
#define CANTASK_H
#include "main.h"

#define CHASSIS_CAN CAN1

#define ARM_CAN CAN2



#define GIMBAL_CAN CAN1

/* CAN send and receive ID */
typedef enum
{
    CAN_CHASSIS_ALL_ID = 0x200,
    CAN_3508_M1_ID = 0x201,
    CAN_3508_M2_ID = 0x202,
    CAN_3508_M3_ID = 0x203,
    CAN_3508_M4_ID = 0x204,
#if ENGINEER_ARM//�ֱ۵��ID
    CAN_3508_M5_ID = 0x201,
    CAN_3508_M6_ID = 0x202,
#endif

    CAN_YAW_MOTOR_ID = 0x205,
    CAN_PIT_MOTOR_ID = 0x206,
    CAN_TRIGGER_MOTOR_ID = 0x207,
    CAN_GIMBAL_ALL_ID = 0x1FF,

} can_msg_id_e;

//rm���ͳһ���ݽṹ��?
typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
} motor_measure_t;

extern void CAN_CMD_CHASSIS_RESET_ID(void);

//������̨�����������revΪ�����ֽ�
extern void CAN_CMD_GIMBAL(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev);
//���͵��̵����������?
extern void CAN_CMD_CHASSIS(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
//����yaw���������ַ���?��ָ�뷽ʽ��ȡԭʼ����
extern const motor_measure_t *get_Yaw_Gimbal_Motor_Measure_Point(void);
//����pitch���������ַ���?��ָ�뷽ʽ��ȡԭʼ����
extern const motor_measure_t *get_Pitch_Gimbal_Motor_Measure_Point(void);
//����trigger���������ַ���?��ָ�뷽ʽ��ȡԭʼ����
extern const motor_measure_t *get_Trigger_Motor_Measure_Point(void);

extern const motor_measure_t *get_Chassis_Motor_Measure_Point(uint8_t i);

#if ENGINEER_ARM
const motor_measure_t* get_Arm_Motor_Measure_Point(uint8_t i);

void CAN_CMD_ARM(int16_t motor1, int16_t motor2);
#endif

#if GIMBAL_MOTOR_6020_CAN_LOSE_SLOVE
extern void GIMBAL_lose_slove(void);
#endif

#endif

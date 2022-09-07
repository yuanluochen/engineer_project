#include "Robot_arm_task.h"
#include "main.h"
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "pid.h"
#include "PID_control_angle.h"
#include "remote_control.h"
#include "chassis_task.h"

static void Robot_arm_init(arm_move_t* arm_move_init);
static void PID_Parameter_config(ExtU_PID_control_angle_T* PID_param_init);
static void Robot_arm_set_mode(arm_move_t* Robot_arm_move_mode);
static void Robot_arm_mode_change_control_transit(arm_move_t* Robot_arm_move_mode);
static void Robot_arm_set_control(arm_move_t* Robot_arm_move_control);
static void Robot_arm_behaviour_control_set(fp32* angle_set, arm_move_t* Robot_arm_move_rc_to_vector);
static void Robot_arm_zero_force_control(fp32* angle_can_set, arm_move_t* Robot_arm_move_rc_vector);
static void Robot_arm_run_control(fp32* angle_can_set, arm_move_t* Robot_arm_move_rc_vector); 
static void Robot_arm_rc_to_control_vector(fp32* angle_set, arm_move_t* Robot_arm_move_rc_to_vector);
static void ARM_control_loop(arm_move_t* Arm_move_control_loop);
static void Arm_vector_to_speed(const fp32 motor2_angle_set, const fp32 motor1_angle_set, fp32 arm_angle[2]);

static arm_move_t arm_move;

//�����ֱۿ��ƺ���
void Robot_arm_task(void* Parameters)
{
    //��ʱһ��ʱ��
    vTaskDelay(357);
    //�ֱ۳�ʼ����Ҫ��ʼ��PID����
    Robot_arm_init(&arm_move);
    while (1)
    {
        //ң��������״̬
        Robot_arm_set_mode(&arm_move);
        //ң���������ϴ�״̬
        Robot_arm_mode_change_control_transit(&arm_move);
        
        //�������ֱۿ�������
        Robot_arm_set_control(&arm_move);
        //�������ֱۿ���PID
        ARM_control_loop(&arm_move);
        //���͵����������
        CAN_CMD_ARM(arm_move.motor_arm[0].give_current, arm_move.motor_arm[1].give_current);
        vTaskDelay(2);
    }
    
    
}

static void Robot_arm_init(arm_move_t* arm_move_init)
{
    
    if (arm_move_init == NULL)
    {
        return;
    }
    int i = 0;
    //���õ��̿���״̬Ϊ����״̬
    arm_move_init->arm_mode = ARM_VECTOR_RAW;
    //��ȡң����ָ��
    arm_move_init->arm_RC = get_remote_control_point();
    //
    const static fp32 chassis_x_order_filter[1] = {ARM_ACCEL_ANGLE_NUM};
    //��ʼ���ٶ�PID,˳���ȡ�ֱ۵������
    for (i = 0; i < ARM_MOTOR_NUM; i++)
    {
        //��ȡ�ֱ۵����ǰ�ٶ�
        arm_move_init->motor_arm->arm_motor_measure = get_Arm_Motor_Measure_Point(i);
        //PID������ֵ
        PID_Parameter_config(arm_move_init->arm_move_pid[i]);
        //PID��ʼ��
        PID_control_angle_initialize(arm_move_init->arm_move_pid[i]);    
    }
    //��һ���˲�����б����������
    first_order_filter_init(&arm_move_init->Robot_arm_cmd_slow_set_angle, ARM_CONTROL_TIME, chassis_x_order_filter);
}

static void PID_Parameter_config(ExtU_PID_control_angle_T* PID_param_init)
{
    if(PID_param_init == NULL)
    {
        return;
    }
    PID_param_init->angle_KP = ARM_PID_ANGLE_KP;
    PID_param_init->angle_KI = ARM_PID_ANGLE_KI;
    PID_param_init->angle_KD = ARM_PID_ANGLE_KD;
    PID_param_init->angle_N = ARM_PID_ANGLE_N;
    PID_param_init->angle_output_up = ARM_PID_ANGLE_MAX;
    PID_param_init->angle_output_low = ARM_PID_ANGLE_MIN;

    PID_param_init->speed_KP = ARM_PID_SPEED_KP;
    PID_param_init->speed_KI = ARM_PID_SPEED_KI;
    PID_param_init->speed_KD = ARM_PID_SPEED_KD;
    PID_param_init->speed_N = ARM_PID_SPEED_N;
    PID_param_init->speed_output_up = ARM_PID_SPEED_MAX;
}

static void Robot_arm_set_mode(arm_move_t* Robot_arm_move_mode)
{
    if(Robot_arm_move_mode == NULL)
    {
        return;
    }
    if (Robot_arm_move_mode == NULL)
    {
        return;
    }
    //��ȡң�������ư���
    if (switch_is_up(Robot_arm_move_mode->arm_RC->rc.s[MODE_ARM]))
    {
        //S2�����ϲ࣬�ֱ۵������
        Robot_arm_move_mode->arm_mode = ARM_VECTOR_YAW;//�ֱ۵��ʹ��״̬
    }
    else
    {
        //S2���������ط����ֱ۵������
        Robot_arm_move_mode->arm_mode = ARM_VECTOR_RAW;
    }
}

static void Robot_arm_mode_change_control_transit(arm_move_t* Robot_arm_move_mode)
{
    if(Robot_arm_move_mode == NULL)
    {
        return;
    }
    //�����ϴ�����
    Robot_arm_move_mode->arm_last_mode = Robot_arm_move_mode->arm_mode;
}

static void Robot_arm_set_control(arm_move_t* Robot_arm_move_control)
{
    if(Robot_arm_move_control == NULL)
    {
        return;
    }
    //��ȡcan����
    fp32 angle_set = 0.0f;
    //��ȡң������������
    Robot_arm_behaviour_control_set(&angle_set, Robot_arm_move_control);
    //���Ƕ��趨ֵ�洢���ṹ����
    Robot_arm_move_control->angle_set = angle_set;   
}

static void Robot_arm_behaviour_control_set(fp32* angle_set, arm_move_t* Robot_arm_move_rc_to_vector)
{
    if(angle_set == NULL || Robot_arm_move_rc_to_vector == NULL)
    {
        return;
    }
    
    if(Robot_arm_move_rc_to_vector->arm_mode == ARM_VECTOR_RAW)
    {
        //��ʱ�ֱ۵��Ϊ����״̬������ֵ����Ϊ��
        Robot_arm_zero_force_control(angle_set, Robot_arm_move_rc_to_vector);
    }
    else if (Robot_arm_move_rc_to_vector->arm_mode == ARM_VECTOR_YAW)
    {
        //���ʹ��
        Robot_arm_run_control(angle_set, Robot_arm_move_rc_to_vector);
    }
    
    
}

static void Robot_arm_zero_force_control(fp32* angle_can_set, arm_move_t* Robot_arm_move_rc_vector)
{
    if(angle_can_set == NULL || Robot_arm_move_rc_vector == NULL)
    {
        return;
    }
    //���÷���can�ĽǶ�ֵΪ0
    *angle_can_set = 0.0f;
}

static void Robot_arm_run_control(fp32* angle_can_set, arm_move_t* Robot_arm_move_rc_to_vector) 
{
    if(angle_can_set == NULL || Robot_arm_move_rc_to_vector == NULL)
    {
        return;
    }
    //��ȡң��������
    Robot_arm_rc_to_control_vector(angle_can_set, Robot_arm_move_rc_to_vector);
}

static void Robot_arm_rc_to_control_vector(fp32* angle_set, arm_move_t* Robot_arm_move_rc_to_vector)
{
    if(angle_set == NULL || Robot_arm_move_rc_to_vector == NULL)
    {
        return;
    }
    //ң����ԭʼͨ��ֵ
    int16_t angle_channel = 0;//�Ƕȿ�����ֵ
    fp32 angle_set_channel = 0;
    //�������ƣ�����ȡͨ��ֵ
    rc_deadline_limit(Robot_arm_move_rc_to_vector->arm_RC->rc.ch[ARM_CONTROL_CHANNEL], angle_channel, ARM_RC_DEADLINE);
    //��ң����ԭʼ����ת��Ϊ�Ƕ��趨�ٶ�
    angle_set_channel = angle_channel * ARM_ANGLE_RC_SEN;
    
    //һ�׵�ͨ�˲�����б����Ϊ�ֱ۵������
    first_order_filter_cali(&Robot_arm_move_rc_to_vector->Robot_arm_cmd_slow_set_angle, angle_set_channel);
    //ֹͣ�źţ�����Ҫ�������٣�ֱ�Ӽ��ٵ���
    if(angle_set_channel < ARM_RC_DEADLINE * ARM_ANGLE_RC_SEN && angle_set_channel > -ARM_RC_DEADLINE * ARM_ANGLE_RC_SEN)
    {
        Robot_arm_move_rc_to_vector->Robot_arm_cmd_slow_set_angle.out = 0.0f;
    }

    *angle_set = Robot_arm_move_rc_to_vector->Robot_arm_cmd_slow_set_angle.out;
    //�����ת���ٶȴ�С��ӵ��ṹ����
}  


static void ARM_control_loop(arm_move_t* Arm_move_control_loop)
{
    int i = 0;
    fp32 arm_angle[2] = {0};
    //����ٶȽ��㣬ʹ��ת���������
    Arm_vector_to_speed(Arm_move_control_loop->angle_set, Arm_move_control_loop->angle_set, arm_angle);
    //�ж�״̬���Ƿ�ʹ�ܵ��
    //����״̬

    if(Arm_move_control_loop->arm_mode == ARM_VECTOR_RAW)
    {
        //��ֵ����ֵ
        for(i = 0; i < ARM_MOTOR_NUM;i++)
        {
            Arm_move_control_loop->motor_arm[i].give_current = (uint16_t)(Arm_move_control_loop->angle_set);
        }
        return;
    }
    //��ֵ����ֵ
    for(i = 0; i < ARM_MOTOR_NUM;i++)
    {
        //PID����
        Arm_move_control_loop->arm_move_pid[i]->angle_set = arm_angle[i];
        Arm_move_control_loop->arm_move_pid[i]->angle_IN = Arm_move_control_loop->motor_arm[i].arm_motor_measure->ecd;
        Arm_move_control_loop->arm_move_pid[i]->speed_IN = Arm_move_control_loop->motor_arm[i].arm_motor_measure->speed_rpm;
        PID_control_angle_step();
        //��ȡPID�����ĵ���ֵ
        Arm_move_control_loop->motor_arm[i].give_current = (uint16_t)Arm_move_control_loop->arm_move_pid_out->angle_OUT;
    }

}

static void Arm_vector_to_speed(const fp32 motor2_angle_set, const fp32 motor1_angle_set, fp32 arm_angle[2])
{
    arm_angle[0] = -motor1_angle_set;
    arm_angle[1] = motor2_angle_set;
}


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

//工程手臂控制函数
void Robot_arm_task(void* Parameters)
{
    //延时一段时间
    vTaskDelay(357);
    //手臂初始化主要初始化PID参数
    Robot_arm_init(&arm_move);
    while (1)
    {
        //遥控器设置状态
        Robot_arm_set_mode(&arm_move);
        //遥控器保存上次状态
        Robot_arm_mode_change_control_transit(&arm_move);
        
        //机器人手臂控制设置
        Robot_arm_set_control(&arm_move);
        //机器人手臂控制PID
        ARM_control_loop(&arm_move);
        //发送电机控制数据
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
    //设置底盘开机状态为无力状态
    arm_move_init->arm_mode = ARM_VECTOR_RAW;
    //获取遥控器指针
    arm_move_init->arm_RC = get_remote_control_point();
    //
    const static fp32 chassis_x_order_filter[1] = {ARM_ACCEL_ANGLE_NUM};
    //初始化速度PID,顺便获取手臂电机数据
    for (i = 0; i < ARM_MOTOR_NUM; i++)
    {
        //获取手臂电机当前速度
        arm_move_init->motor_arm->arm_motor_measure = get_Arm_Motor_Measure_Point(i);
        //PID参数赋值
        PID_Parameter_config(arm_move_init->arm_move_pid[i]);
        //PID初始化
        PID_control_angle_initialize(arm_move_init->arm_move_pid[i]);    
    }
    //用一阶滤波代替斜波函数生成
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
    //获取遥控器控制按键
    if (switch_is_up(Robot_arm_move_mode->arm_RC->rc.s[MODE_ARM]))
    {
        //S2拨到上侧，手臂电机驱动
        Robot_arm_move_mode->arm_mode = ARM_VECTOR_YAW;//手臂电机使能状态
    }
    else
    {
        //S2拨到其他地方，手臂电机无力
        Robot_arm_move_mode->arm_mode = ARM_VECTOR_RAW;
    }
}

static void Robot_arm_mode_change_control_transit(arm_move_t* Robot_arm_move_mode)
{
    if(Robot_arm_move_mode == NULL)
    {
        return;
    }
    //保存上次数据
    Robot_arm_move_mode->arm_last_mode = Robot_arm_move_mode->arm_mode;
}

static void Robot_arm_set_control(arm_move_t* Robot_arm_move_control)
{
    if(Robot_arm_move_control == NULL)
    {
        return;
    }
    //获取can数据
    fp32 angle_set = 0.0f;
    //获取遥控器设置数据
    Robot_arm_behaviour_control_set(&angle_set, Robot_arm_move_control);
    //将角度设定值存储到结构体中
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
        //此时手臂电机为无力状态，所有值设置为零
        Robot_arm_zero_force_control(angle_set, Robot_arm_move_rc_to_vector);
    }
    else if (Robot_arm_move_rc_to_vector->arm_mode == ARM_VECTOR_YAW)
    {
        //电机使能
        Robot_arm_run_control(angle_set, Robot_arm_move_rc_to_vector);
    }
    
    
}

static void Robot_arm_zero_force_control(fp32* angle_can_set, arm_move_t* Robot_arm_move_rc_vector)
{
    if(angle_can_set == NULL || Robot_arm_move_rc_vector == NULL)
    {
        return;
    }
    //设置发送can的角度值为0
    *angle_can_set = 0.0f;
}

static void Robot_arm_run_control(fp32* angle_can_set, arm_move_t* Robot_arm_move_rc_to_vector) 
{
    if(angle_can_set == NULL || Robot_arm_move_rc_to_vector == NULL)
    {
        return;
    }
    //获取遥控器数据
    Robot_arm_rc_to_control_vector(angle_can_set, Robot_arm_move_rc_to_vector);
}

static void Robot_arm_rc_to_control_vector(fp32* angle_set, arm_move_t* Robot_arm_move_rc_to_vector)
{
    if(angle_set == NULL || Robot_arm_move_rc_to_vector == NULL)
    {
        return;
    }
    //遥控器原始通道值
    int16_t angle_channel = 0;//角度控制数值
    fp32 angle_set_channel = 0;
    //死区限制，并获取通道值
    rc_deadline_limit(Robot_arm_move_rc_to_vector->arm_RC->rc.ch[ARM_CONTROL_CHANNEL], angle_channel, ARM_RC_DEADLINE);
    //将遥控器原始数据转换为角度设定速度
    angle_set_channel = angle_channel * ARM_ANGLE_RC_SEN;
    
    //一阶低通滤波代替斜波作为手臂电机输入
    first_order_filter_cali(&Robot_arm_move_rc_to_vector->Robot_arm_cmd_slow_set_angle, angle_set_channel);
    //停止信号，不需要缓慢加速，直接减速到零
    if(angle_set_channel < ARM_RC_DEADLINE * ARM_ANGLE_RC_SEN && angle_set_channel > -ARM_RC_DEADLINE * ARM_ANGLE_RC_SEN)
    {
        Robot_arm_move_rc_to_vector->Robot_arm_cmd_slow_set_angle.out = 0.0f;
    }

    *angle_set = Robot_arm_move_rc_to_vector->Robot_arm_cmd_slow_set_angle.out;
    //将电机转动速度大小添加到结构体内
}  


static void ARM_control_loop(arm_move_t* Arm_move_control_loop)
{
    int i = 0;
    fp32 arm_angle[2] = {0};
    //电机速度解算，使其转动方向相对
    Arm_vector_to_speed(Arm_move_control_loop->angle_set, Arm_move_control_loop->angle_set, arm_angle);
    //判断状态，是否使能电机
    //无力状态

    if(Arm_move_control_loop->arm_mode == ARM_VECTOR_RAW)
    {
        //赋值电流值
        for(i = 0; i < ARM_MOTOR_NUM;i++)
        {
            Arm_move_control_loop->motor_arm[i].give_current = (uint16_t)(Arm_move_control_loop->angle_set);
        }
        return;
    }
    //赋值电流值
    for(i = 0; i < ARM_MOTOR_NUM;i++)
    {
        //PID运算
        Arm_move_control_loop->arm_move_pid[i]->angle_set = arm_angle[i];
        Arm_move_control_loop->arm_move_pid[i]->angle_IN = Arm_move_control_loop->motor_arm[i].arm_motor_measure->ecd;
        Arm_move_control_loop->arm_move_pid[i]->speed_IN = Arm_move_control_loop->motor_arm[i].arm_motor_measure->speed_rpm;
        PID_control_angle_step();
        //获取PID解算后的电流值
        Arm_move_control_loop->motor_arm[i].give_current = (uint16_t)Arm_move_control_loop->arm_move_pid_out->angle_OUT;
    }

}

static void Arm_vector_to_speed(const fp32 motor2_angle_set, const fp32 motor1_angle_set, fp32 arm_angle[2])
{
    arm_angle[0] = -motor1_angle_set;
    arm_angle[1] = motor2_angle_set;
}


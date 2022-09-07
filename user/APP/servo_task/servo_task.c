#include "servo_task.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "remote_control.h"

static servo_move_t servo_move; 
//死区限制
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


static void servo_init(servo_move_t* servo_move_init);
static void servo_tim_init(void);
static void servo_set_mode(servo_move_t* servo_move_mode);
static void servo_mode_change_control_transit(servo_move_t* servo_move_transit);
static void servo_set_control(servo_move_t* servo_move_control);
static void servo_behaviour_control_set(fp32* angle_set, servo_move_t* servo_RC_to_angle_control);
static void servo_state_enable_control(fp32* angle_set, servo_move_t* servo_RC_to_angle_control);
static void servo_angle_set_transmit(servo_move_t* servo_angle_set_transmit);

void servo_task( void * pvParameters)
{
    //空闲一段时间
    vTaskDelay(SERVO_TASK_INIT_TIME);
    //servo 初始化
    servo_init(&servo_move);
    while(1)
    {
        //遥控器设置舵机状态
        servo_set_mode(&servo_move);
        //遥控器状态切换数据保存
        servo_mode_change_control_transit(&servo_move);
        //获取遥控器数据，从遥控器中获取舵机角度数据
        servo_set_control(&servo_move);
        //发送电机角度控制角度数据
        servo_angle_set_transmit(&servo_move);
        vTaskDelay(SERVO_CONTROL_TIME_MS);   
    }
}

static void servo_init(servo_move_t* servo_move_init)
{
    servo_tim_init();
    const static fp32 servo_angle_order_filter[1] = {SERVO_ACCEL_ANGLE_NUM};
    //设置舵机模式，配置舵机初始模式为无力
    servo_move_init->servo_mode = SERVO_STATE_DISABLE;
    //获取遥控器指针
    servo_move_init->servo_RC = get_remote_control_point();
    //初始化滤波函数，用一阶滤波代替斜波函数
    first_order_filter_init(&servo_move_init->servo_cmd_slow_set_angle, SERVO_CONTROL_TIME, servo_angle_order_filter);

    //配置最大角度
    servo_move_init->angle_max = SERVO_ANGLE_180;
    servo_move_init->angle_min = SERVO_ANGLE_0;
}


//配置PA3 PWM
static void servo_tim_init(void)
{   
    //TIM2 CH4
    //PA3
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_InitStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_TIM2);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /*
     * APB1总线 42MHz 4200 分频 10000Hz 每计数一次时间为0.1ms
     * 舵机转动角度
     * | angle | HIGH_Level_Time | ARR
     * | 0     | 0.5ms           | 5
     * | 45    | 1.0ms           | 10
     * | 90    | 1.5ms           | 15
     * | 135   | 2.0ms           | 20
     * | 180   | 2.5ms           | 25
    */
    TIM_InitStructure.TIM_Prescaler = 4199;//APB1 总线 42MHz
    TIM_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;//向上计数
    TIM_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_InitStructure.TIM_Period = SERVO_ANGLE_45;
    TIM_TimeBaseInit(TIM2, &TIM_InitStructure);
    //PWM1 mode 
    //CounterMode_Up 
    //TIMx_CNT < TIMx_CCR 此时为有效电平，其余时间为无效电平
    //CounterMode_down
    //TIMx_CNT < TIMx_CCR 此时为无效电平，其余时间为有效电平
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;//有效时间为正电平
    TIM_OC4Init(TIM2, &TIM_OCInitStructure);

    TIM_CtrlPWMOutputs(TIM2, ENABLE);
    TIM_Cmd(TIM2, ENABLE);//TIM2时钟使能

    //舵机回到0度
    TIM_SetCompare4(TIM2, SERVO_ANGLE_0);    
}

static void servo_set_mode(servo_move_t* servo_move_mode)
{
    if(servo_move_mode == NULL)
    {
        return;
    }

    if(switch_is_up(servo_move_mode->servo_RC->rc.s[MODE_CHANNEL]))
    {
        //左侧拨于上侧，舵机使能
        servo_move_mode->servo_mode = SERVO_STATE_ENABLE;
    }
    else if(!switch_is_up(servo_move_mode->servo_RC->rc.s[MODE_CHANNEL]))
    {
        //左侧拨到其他位置，舵机无力
        servo_move_mode->servo_mode = SERVO_STATE_DISABLE;
    }
    
}
static void servo_mode_change_control_transit(servo_move_t* servo_move_transit)
{
    if(servo_move_transit == NULL)
    {
        return;
    }

    //前后模式相同不保存原始数据
    if(servo_move_transit->last_servo_mode == servo_move_transit->servo_mode)
    {
        return;
    }

    //保存上次数据
    servo_move_transit->last_servo_mode = servo_move_transit->servo_mode;
}

static void servo_set_control(servo_move_t* servo_move_control)
{
    if(servo_move_control == NULL)
    {
        return;
    }
    fp32 angle_set = 0.0f;
    //获取遥控器数据
    servo_behaviour_control_set(&angle_set, servo_move_control);
    //将角度设置数值赋值给结构体角度设置变量内，此时角度数值为定时器自动重装载值。
    servo_move_control->angle_set = angle_set;

}

static void servo_behaviour_control_set(fp32* angle_set, servo_move_t* servo_RC_to_angle_control)
{
    if(angle_set == NULL || servo_RC_to_angle_control == NULL)
    {
        return;
    }
    if(servo_RC_to_angle_control->servo_mode == SERVO_STATE_DISABLE)
    {
        *angle_set = 0.0f;//舵机无力状态下，舵机不进行运动
    }
    else if(servo_RC_to_angle_control->servo_mode == SERVO_STATE_ENABLE)
    {
        servo_state_enable_control(angle_set, servo_RC_to_angle_control);
    }

}

static void servo_state_enable_control(fp32* angle_set, servo_move_t* servo_RC_to_angle_control)
{
    if(angle_set == NULL || servo_RC_to_angle_control == NULL)
    {
        return;
    }
    int16_t angle_channel;//遥控器原始值
    fp32 angle_set_channel;//舵机角度设置值
    fp32 servo_tim_arr;
    //死区限制
    rc_deadline_limit(servo_RC_to_angle_control->servo_RC->rc.ch[SERVO_CONTROL_CHANNEL], angle_channel, SERVO_CONTROL_CHANNEL);
    //将遥控器原始数据转换为舵机角度设置值
    angle_set_channel = angle_channel * SERVO_ANGLE_RC_SEN;
    //将角度值转化为PWD自动重装载值
    servo_tim_arr = angle_set_channel * SERVO_ANGLE_TO_ARR;

    //一阶低通滤波代替斜波作为舵机角度输入
    first_order_filter_cali(&servo_RC_to_angle_control->servo_cmd_slow_set_angle, servo_tim_arr);
    //得到滤波数值，将数值赋值给angle_set
    *angle_set = servo_RC_to_angle_control->servo_cmd_slow_set_angle.out;
}
static void servo_angle_set_transmit(servo_move_t* servo_angle_set_transmit)
{
    TIM_SetCompare4(TIM2, (uint32_t)servo_angle_set_transmit->angle_set);
}

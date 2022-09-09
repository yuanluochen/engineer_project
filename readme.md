该工程原为DJI官方2019年官方步兵开源代码，由yuanluochen更改为RM工程机器人代码，手臂电机利用can2发送，ID号为0x201,0x202,头部摄像机移动舵机信号引脚为PA3，TIM2
目录：
CMSIS：内核相关的文件
FWLIB：标准库文件
Project：工程文件
startup:芯片启动文件
user：用户编写的相关文件，主要编写的文件都在这个文件夹下

user/main.c\h :main函数所在的文件
user/AHRS：陀螺仪驱动以及姿态解算
user/APP：freeRTOS任务
user/DSP：DSP库
user/FreeRTOS:移植的freeRTOS文件
user/hardware：硬件层
user/user_lib：编写的数学函数

# 用户记录

舵机 servo_move 结构体内部在获取遥控器数据时，将遥控器数值转化为舵机转动角度，将舵机转动角度转化为定时器自动重装载值，该变量存储在结构体成员变量angle_set。

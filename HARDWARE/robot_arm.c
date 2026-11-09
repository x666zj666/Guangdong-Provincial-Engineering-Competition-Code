#include "robot_arm.h"

// 机械臂状态变量
float Current_Angle[6] = {0};
float Taget_Angle[6] = {0};
uint8_t arm_task_enable = 0;

short Angle1_init=0,Angle2_init=0,Angle3_init=0,Angle4_init=0,Angle5_init=0,Angle6_init=0; //舵机初始位置值

//机械臂舵机相关变量
ServoController Servo1 = {135, 135, 0, 0.03, 4.0}; // 加速度0.01，最大速度4
ServoController Servo2 = {222,222, 0, 0.05, 5.0};
ServoController Servo3 = {270, 270, 0, 0.04, 5.0}; // 加速度0.01，最大速度5
ServoController Servo4 = {197, 197, 0, 0.05, 5.0};
ServoController Servo5 = {90, 90, 0, 0.04, 5.0}; // 加速度0.01，最大速度5
ServoController Servo6 = {90, 90, 0, 0.04, 5.0};

/**************************************************************************
Function: 机械臂PWM初始化
Input   : none
Output  : none
功能说明：初始化6个舵机的PWM输出，使用TIM8和TIM12定时器
引脚配置：PC6 PC7 PC8 PC9 PB14 PB15
**************************************************************************/
void RobotArm_PWM_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;
    
    // 使能时钟 - TIM8需要APB2时钟，TIM12需要APB1时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);  // TIM8在APB2
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12, ENABLE); // TIM12在APB1
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOB, ENABLE);
    
    // 配置GPIO为复用功能
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    
    // PC6, PC7, PC8, PC9 - TIM8 CH1, CH2, CH3, CH4
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM8);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM8);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_TIM8);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_TIM8);
    
    // PB14, PB15 - TIM12 CH1, CH2
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_TIM12);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_TIM12);
    
    // 配置TIM8 (舵机1,2,3,4) - PC6,PC7,PC8,PC9
    TIM_TimeBaseStructure.TIM_Period = SERVO_PWM_PERIOD - 1;
    TIM_TimeBaseStructure.TIM_Prescaler = SERVO_PWM_PRESCALER_TIM8 - 1;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;  // TIM8需要设置重复计数器
    TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);
    
    
    TIM_OCStructInit(&TIM_OCInitStructure);    //必须补上，不然TIM8输出波形会反向！！！！！！！！！
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
//    TIM_OCInitStructure.TIM_Pulse = SERVO_PWM_CENTER;  //初始脉宽
    TIM_OCInitStructure.TIM_Pulse = 0;  //初始脉宽
    
    // TIM8的4个通道
    TIM_OC1Init(TIM8, &TIM_OCInitStructure);
    TIM_OC2Init(TIM8, &TIM_OCInitStructure);
    TIM_OC3Init(TIM8, &TIM_OCInitStructure);
    TIM_OC4Init(TIM8, &TIM_OCInitStructure);
    
    TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(TIM8, TIM_OCPreload_Enable);
    
    
    TIM_CtrlPWMOutputs(TIM8, ENABLE);  // TIM8需要使能PWM输出
    
    // 配置TIM12 (舵机5,6) - PB14,PB15
    TIM_TimeBaseStructure.TIM_Prescaler = SERVO_PWM_PRESCALER_TIM12 - 1;
    TIM_TimeBaseInit(TIM12, &TIM_TimeBaseStructure);
    TIM_OC1Init(TIM12, &TIM_OCInitStructure);
    TIM_OC2Init(TIM12, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM12, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM12, TIM_OCPreload_Enable);
    
    TIM_Cmd(TIM8, ENABLE);
    TIM_Cmd(TIM12, ENABLE);
}

/**************************************************************************
Function: 机械臂初始化
Input   : none
Output  : none
功能说明：初始化机械臂系统
**************************************************************************/
void RobotArm_Init(void)
{
    RobotArm_PWM_Init();
    
    //设置所有舵机的初始速度参数
    SetServo_AccelSpeed(&Servo1,0.04,5.0);
    SetServo_AccelSpeed(&Servo2,0.08,5.0);
    SetServo_AccelSpeed(&Servo3,0.055,5.0);
    SetServo_AccelSpeed(&Servo4,0.05,5.0);
    SetServo_AccelSpeed(&Servo5,0.04,5.0);
    SetServo_AccelSpeed(&Servo6,0.04,5.0);
  
    arm_task_enable = 1;
}

/**************************************************************************
Function: 角度限制函数
Input   : servo_num - 舵机编号(1-6), angle - 目标角度
Output  : 限制后的角度
功能说明：根据舵机类型限制角度范围
**************************************************************************/
float RobotArm_LimitAngle(uint8_t servo_num, float angle)
{
    switch(servo_num) {
        case 1:
            if(angle < SERVO_1_MIN_ANGLE) angle = SERVO_1_MIN_ANGLE;
            if(angle > SERVO_1_MAX_ANGLE) angle = SERVO_1_MAX_ANGLE;
            break;
        case 2:
            if(angle < SERVO_2_MIN_ANGLE) angle = SERVO_2_MIN_ANGLE;
            if(angle > SERVO_2_MAX_ANGLE) angle = SERVO_2_MAX_ANGLE;
            break;
        case 3:
            if(angle < SERVO_3_MIN_ANGLE) angle = SERVO_3_MIN_ANGLE;
            if(angle > SERVO_3_MAX_ANGLE) angle = SERVO_3_MAX_ANGLE;
            break;
        case 4:
            if(angle < SERVO_4_MIN_ANGLE) angle = SERVO_4_MIN_ANGLE;
            if(angle > SERVO_4_MAX_ANGLE) angle = SERVO_4_MAX_ANGLE;
            break;
        case 5:
            if(angle < SERVO_5_MIN_ANGLE) angle = SERVO_5_MIN_ANGLE;
            if(angle > SERVO_5_MAX_ANGLE) angle = SERVO_5_MAX_ANGLE;
            break;
        case 6:
            if(angle < SERVO_6_MIN_ANGLE) angle = SERVO_6_MIN_ANGLE;
            if(angle > SERVO_6_MAX_ANGLE) angle = SERVO_6_MAX_ANGLE;
            break;
        default:
            break;
    }
    return angle;
}

/**************************************************************************
Function: 角度转PWM值
Input   : angle - 角度值
          Range_Angle - 角度范围
Output  : PWM值
功能说明：将角度转换为PWM占空比值
**************************************************************************/
uint16_t servo_angle_transform_PWM(float angle,float Range_Angle)
{
    uint16_t pwm_value;
    
    // 将角度映射到PWM范围 (500-2500)
    pwm_value = (uint16_t)(angle / Range_Angle * (SERVO_PWM_MAX - SERVO_PWM_MIN) + SERVO_PWM_MIN);
    
    // 限制PWM范围
    if(pwm_value < SERVO_PWM_MIN) pwm_value = SERVO_PWM_MIN;
    if(pwm_value > SERVO_PWM_MAX) pwm_value = SERVO_PWM_MAX;
    
    return pwm_value;
}

/**************************************************************************
Function: 设置单个舵机角度
Input   : servo_num - 舵机编号(1-6), angle - 角度值
Output  : none
功能说明：输入角度值直接控制舵机，自动进行角度限制和PWM转换(连续调用，用于平滑运动)
**************************************************************************/
void RobotArm_Set_CurrentAngle(uint8_t servo_num, float angle)
{
    if(servo_num < 1 || servo_num > 6) {
        return;
    }
    // 角度限制
    float limited_angle = RobotArm_LimitAngle(servo_num, angle);
    // 转换为PWM值并且设置PWM
    switch(servo_num) {
        case 1: SERVO_1_PWM=servo_angle_transform_PWM(limited_angle,SERVO_1_RANGE_ANGLE)+Angle1_init; break;
        case 2: SERVO_2_PWM=servo_angle_transform_PWM(limited_angle,SERVO_2_RANGE_ANGLE)+Angle2_init; break;
        case 3: SERVO_3_PWM=servo_angle_transform_PWM(limited_angle,SERVO_3_RANGE_ANGLE)+Angle3_init; break;
        case 4: SERVO_4_PWM=servo_angle_transform_PWM(limited_angle,SERVO_4_RANGE_ANGLE)+Angle4_init; break;
        case 5: SERVO_5_PWM=servo_angle_transform_PWM(limited_angle,SERVO_5_RANGE_ANGLE)+Angle5_init; break;
        case 6: SERVO_6_PWM=servo_angle_transform_PWM(limited_angle,SERVO_6_RANGE_ANGLE)+Angle6_init; break;
    }
    // 更新当前位置
    switch(servo_num) {
        case 1: Current_Angle[0] = limited_angle; break;
        case 2: Current_Angle[1] = limited_angle; break;
        case 3: Current_Angle[2] = limited_angle; break;
        case 4: Current_Angle[3] = limited_angle; break;
        case 5: Current_Angle[4] = limited_angle; break;
        case 6: Current_Angle[5] = limited_angle; break;
    }
}

/**************************************************************************
Function: 设置单个舵机的目标角度
Input   : servo_num - 舵机编号(1-6), angle - 目标角度
Output  : none
功能说明：设置目标角度，具有角度限制
**************************************************************************/
void RobotArm_Set_TargetAngle(uint8_t servo_num, float angle)
{
  if(servo_num < 1 || servo_num > 6) {
        return;
    }
  
    angle = RobotArm_LimitAngle(servo_num, angle);
    switch(servo_num) {
        case 1:
            Servo1.target_angle=angle;
            Taget_Angle[0] = angle;
            break;
        case 2:
            Servo2.target_angle=angle;
            Taget_Angle[1] = angle;
            break;
        case 3:
            Servo3.target_angle=angle;
            Taget_Angle[2] = angle;
            break;
        case 4:
            Servo4.target_angle=angle;
            Taget_Angle[3] = angle;
            break;
        case 5:
            Servo5.target_angle=angle;
            Taget_Angle[4] = angle;
            break;
        case 6:
            Servo6.target_angle=angle;
            Taget_Angle[5] = angle;
            break;
        default:
            break;
    }
}

void set_target_angle(ServoController *servo, float target) //不具有角度限制
{
    servo->target_angle = target;
}


/**************************************************************************
Function: 设置所有舵机角度
Input   : angle1-angle6 - 6个舵机的角度值
Output  : none
功能说明：一次性设置所有6个舵机的角度    ——输入-1表示不改变值
**************************************************************************/
void RobotArm_SetAllTargetAngles(float angle1, float angle2, float angle3, 
                          float angle4, float angle5, float angle6)
{
    Servo1.target_angle=angle1;
    Servo2.target_angle=angle2;
    Servo3.target_angle=angle3;
    Servo4.target_angle=angle4;
    Servo5.target_angle=angle5;
    if(angle6!=-1)Servo6.target_angle=angle6;
  
    Taget_Angle[0] = angle1;
    Taget_Angle[1] = angle2;
    Taget_Angle[2] = angle3;
    Taget_Angle[3] = angle4;
    Taget_Angle[4] = angle5;
    if(angle6!=-1)Taget_Angle[5] = angle6;
}

/**************************************************************************
Function: 舵机加速度与速度数值调整
Input   : *servo - 舵机结构体指针,accel - 新加速度, speed - 新最大速度
Output  : none
功能说明：单个舵机的加速度与最大加速度数值的改变
**************************************************************************/
void SetServo_AccelSpeed(ServoController *servo,float accel,float speed)  //舵机数值调整
{
  if(accel<0)accel=0;
  if(speed<0)speed=0;
  servo->accel_step=accel;
  servo->max_speed=speed;
}

/**************************************************************************
Function: 平滑移动舵机
Input   : servo_num - 舵机编号, *servo - 舵机结构体指针
Output  : none
功能说明：平滑移动舵机到目标角度，避免突然动作;更新单个舵机的平滑运动（需周期性调用，例如10ms）
**************************************************************************/
void update_smooth_servo(uint8_t servo_num,ServoController *servo) {
    float remaining = servo->target_angle - servo->current_angle;
    
    // 计算目标方向
    float direction = (remaining > 0) ? 1.0 : -1.0;
    
    // 梯形速度规划
    if (fabs(remaining) > 0.1) { // 死区避免抖动
        // 加速阶段
        if (servo->current_step < servo->max_speed && 
            fabs(remaining) > (servo->current_step * servo->current_step) / (2 * servo->accel_step)) {
            servo->current_step += servo->accel_step;
        }
        // 减速阶段
        else if (fabs(remaining) < (servo->current_step * servo->current_step) / (2 * servo->accel_step)) {
            servo->current_step -= servo->accel_step;
            if (servo->current_step < 0) servo->current_step = 0;
        }
        
        // 限制最大速度
        if (servo->current_step > servo->max_speed) {
            servo->current_step = servo->max_speed;
        }
        
        // 更新当前位置
        servo->current_angle += direction * servo->current_step;
    } else {
        servo->current_step = 0;
        servo->current_angle = servo->target_angle; // 到达目标
    }
    
    RobotArm_Set_CurrentAngle(servo_num,servo->current_angle);//输出PWM波形
}
/**************************************************************************
Function: 角度刷新函数
Input   : none
Output  : none
功能说明：放于任务函数中周期调用
**************************************************************************/
void Update_Servo(void)
{
  update_smooth_servo(1,&Servo1);
  update_smooth_servo(2,&Servo2);
  update_smooth_servo(3,&Servo3);
  update_smooth_servo(4,&Servo4);
  update_smooth_servo(5,&Servo5);
  update_smooth_servo(6,&Servo6);
}

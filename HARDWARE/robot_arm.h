#ifndef __ROBOT_ARM_H
#define __ROBOT_ARM_H
#include "system.h"

// 机械臂舵机定义 - 优化后的引脚配置
// 第1个舵机：270度舵机，水平旋转
#define SERVO_1_PWM    TIM8->CCR1   // PC6
#define SERVO_1_TIM     TIM8

// 第2个舵机：270度舵机，控制机械臂高度
#define SERVO_2_PWM    TIM8->CCR2   // PC7  
#define SERVO_2_TIM     TIM8

// 第3个舵机：270度舵机，控制水平夹取位置
#define SERVO_3_PWM    TIM8->CCR3   // PC8
#define SERVO_3_TIM     TIM8

// 第4个舵机：180度舵机，小幅度控制水平夹取位置
#define SERVO_4_PWM    TIM8->CCR4   // PC9
#define SERVO_4_TIM     TIM8

// 第5个舵机：180度舵机，控制机械臂前段夹子旋转
#define SERVO_5_PWM    TIM12->CCR1  // PB14
#define SERVO_5_TIM     TIM12

// 第6个舵机：180度舵机，控制夹子开合
#define SERVO_6_PWM    TIM12->CCR2  // PB15
#define SERVO_6_TIM     TIM12

// 舵机角度范围定义
#define SERVO_1_RANGE_ANGLE    270.0f  //270度舵机
#define SERVO_1_MIN_ANGLE    0.0f    // 限制最小角度
#define SERVO_1_MAX_ANGLE    270.0f  // 限制最大角度
#define SERVO_1_CENTER_ANGLE  135.0f

#define SERVO_2_RANGE_ANGLE    270.0f  // 270度舵机
#define SERVO_2_MIN_ANGLE    0.0f    
#define SERVO_2_MAX_ANGLE    270.0f
#define SERVO_2_CENTER_ANGLE  135.0f

#define SERVO_3_RANGE_ANGLE    270.0f  // 270度舵机
#define SERVO_3_MIN_ANGLE    0.0f    
#define SERVO_3_MAX_ANGLE    270.0f
#define SERVO_3_CENTER_ANGLE  135.0f

#define SERVO_4_RANGE_ANGLE    270.0f  // 270度舵机
#define SERVO_4_MIN_ANGLE    0.0f     
#define SERVO_4_MAX_ANGLE    270.0f
#define SERVO_4_CENTER_ANGLE  135.0f

#define SERVO_5_RANGE_ANGLE    180.0f  // 180度舵机
#define SERVO_5_MIN_ANGLE    0.0f    
#define SERVO_5_MAX_ANGLE    180.0f
#define SERVO_5_CENTER_ANGLE  90.0f

#define SERVO_6_RANGE_ANGLE    180.0f  // 180度舵机
#define SERVO_6_MIN_ANGLE    0.0f    
#define SERVO_6_MAX_ANGLE    180.0f
#define SERVO_6_CENTER_ANGLE  90.0f

// PWM参数定义
#define SERVO_PWM_PERIOD     20000   // 20ms周期
#define SERVO_PWM_PRESCALER_TIM12  84      // 84MHz/84 = 1MHz, 1MHz/20000 = 50Hz
#define SERVO_PWM_PRESCALER_TIM8  168      // 84MHz/84 = 1MHz, 1MHz/20000 = 50Hz
#define SERVO_PWM_MIN        500     // 0.5ms脉宽
#define SERVO_PWM_MAX        2500    // 2.5ms脉宽
#define SERVO_PWM_CENTER     1500    // 1.5ms脉宽
#define SERVO_INIT 1500

#define BOLL_CATCH Servo6.target_angle = 125
#define HOSTAGE_CATCH Servo6.target_angle = 117
#define JIAZI_RELEASE Servo6.target_angle = 54
//舵机状态结构体
typedef struct {
    float current_angle;   // 当前角度
    float target_angle;    // 目标角度
    float current_step;    // 当前速度（步长）
    float accel_step;      // 加速度
    float max_speed;       // 最大速度
} ServoController;

// 机械臂状态定义
//typedef enum {
//    ARM_IDLE = 0,           // 空闲状态
//    ARM_INIT,              // 初始化状态
//    ARM_PICK_UP,           // 抓取状态
//    ARM_PLACE_DOWN,        // 放置状态
//    ARM_RESET,             // 复位状态
//    ARM_MANUAL_CONTROL     // 手动控制状态
//} ArmState_t;

// 机械臂位置结构体
//typedef struct {
//    float servo1_angle;    // 水平旋转角度
//    float servo2_angle;    // 高度控制角度
//    float servo3_angle;    // 水平夹取位置角度
//    float servo4_angle;    // 小幅度水平夹取角度
//    float servo5_angle;    // 夹子旋转角度
//    float servo6_angle;    // 夹子开合角度
//} ArmPosition_t;

// 机械臂预设位置
//typedef enum {
//    ARM_POS_HOME = 0,      // 初始位置
//    ARM_POS_PICK,          // 抓取位置
//    ARM_POS_PLACE,         // 放置位置
//    ARM_POS_STORAGE,       // 存储位置
//    ARM_POS_SCAN           // 扫描位置
//} ArmPresetPosition_t;

// 外部变量声明
//extern ArmState_t arm_state;
//extern ArmPosition_t current_arm_pos;
//extern ArmPosition_t target_arm_pos;
extern uint8_t arm_task_enable;

//机械臂舵机相关变量
extern ServoController Servo1;
extern ServoController Servo2;
extern ServoController Servo3;
extern ServoController Servo4;
extern ServoController Servo5;
extern ServoController Servo6;
extern short Angle1_init,Angle2_init,Angle3_init,Angle4_init,Angle5_init,Angle6_init; //舵机初始位置值
extern float Current_Angle[6];
extern float Taget_Angle[6];

// 初始化函数
void RobotArm_Init(void);
void RobotArm_PWM_Init(void);


// 角度控制函数
void RobotArm_Set_CurrentAngle(uint8_t servo_num, float angle);
void RobotArm_Set_TargetAngle(uint8_t servo_num, float angle);
void RobotArm_SetAllTargetAngles(float angle1, float angle2, float angle3, 
                                 float angle4, float angle5, float angle6);
void set_target_angle(ServoController *servo, float target);
// 角度限制函数
float RobotArm_LimitAngle(uint8_t servo_num, float angle);

// 角度转换函数
uint16_t servo_angle_transform_PWM(float angle,float Range_Angle);

//速度平滑函数
void update_smooth_servo(uint8_t servo_num,ServoController *servo);
void SetServo_AccelSpeed(ServoController *servo,float accel,float speed);
//外部刷新函数
void Update_Servo(void);
//外部调用函数


#endif

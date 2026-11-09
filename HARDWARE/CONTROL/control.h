#ifndef __CONTROL_H
#define __CONTROL_H
#include "system.h"

//int EXTI15_10_IRQHandler(void);
//void Key(void);
void Servo_angle_check(void);
//void Servo_Pwm_check(void);
//void Servo_Speed_Check(void);

//void Get_Angle(u8 way);
//int SERVO_PWM_VALUE_180(float angle);
//int SERVO_PWM_VALUE_270(float angle);

//	float PWM_Angle_VALUE(int pwm);
//float Position_PID1 (float Encoder,float Target);
//float Position_PID2 (float Encoder,float Target);
//float Position_PID3 (float Encoder,float Target);
//float Position_PID4 (float Encoder,float Target);
//float Position_PID5 (float Encoder,float Target);
//float Position_PID6 (float Encoder,float Target);
//void Set_Pwm(float velocity1,float velocity2,float velocity3,float velocity4,float velocity5,float velocity6);
void Servo_init_angle_adjust_function(void);
void Servo_init_angle_adjust_mode_check(void);
void Servo_init_pose(u8 j1,u8 j2,u8 j3,u8 j4,u8 j5,u8 j6);
//void CAN_Control(void);
void save_init_pose(void);
void ik_kinematics(float x,float y,float z,float rx,float ry);
void fk_kinematics(float angle1,float angle2,float angle3,float angle4,float angle5);
u8 RobotArm_Control(float x,float y,float z,float rx,float ry);
u8 Robot_SetAngle(float aa1,float aa2,float aa3,float aa4,float aa5); //正解
float deg2rad(float degrees); //角度转换弧度
  float rad2deg(float radians); //弧度转换角度
extern float Px,Py,Pz,Rx,Ry;

extern u8 mode;
extern u8 ik_flag;//逆解标志位，有解为1，无解为0

//extern int Servo1,Servo2,Servo3,Servo4,Servo5,Servo6;
//extern float Position1,Position2,Position3,Position4,Position5,Position6; //PID控制相关变量    
//extern float Velocity1,Velocity2,Velocity3,Velocity4,Velocity5,Velocity6;     //电机PWM变量
extern float move_angle[7];
//extern short Angle1_init,Angle2_init,Angle3_init,Angle4_init,Angle5_init,Angle6_init;
#endif

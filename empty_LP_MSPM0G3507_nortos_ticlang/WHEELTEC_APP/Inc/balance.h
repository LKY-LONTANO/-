#ifndef __BALANCE_H
#define __BALANCE_H

#include "ti_msp_dl_config.h"
#include <stdlib.h>


extern float Balance_Kp,Balance_Kd,Velocity_Kp,Velocity_Ki,Turn_Kp,Turn_Kd;
extern uint8_t Flag_front,Flag_back,Flag_Left,Flag_Right,Flag_velocity;
extern short MiddleAngle;
extern int Motor_Left,Motor_Right;
extern float Velocity_Left,Velocity_Right;
extern uint8_t Flag_Stop;
extern float robotVol;
//平衡控制任务
void BalanceControlTask(void);

//小车的参数,只是用于计算车轮速度显示,未用于小车的控制
#define PI 3.14159265f	
#define Control_Frequency  100.0
#define Diameter_67  67.0 
#define EncoderMultiples   2.0
#define Encoder_precision  13.0 
#define Reduction_Ratio  30.0	
#define Perimeter  210.4867 	

//内部使用函数
static uint16_t Get_Vol(void);
static void set_pwm(int left,int right);
static void Get_Velocity_Form_Encoder(int encoder_left,int encoder_right);
static int PWM_Limit(int IN,int max,int min);
static int Balance(float Angle,float Gyro);
static int Velocity(int encoder_left,int encoder_right);
static int Turn(float gyro);
#endif

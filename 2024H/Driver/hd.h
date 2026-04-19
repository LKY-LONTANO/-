#ifndef HD_H_
#define HD_H_

#include "ti_msp_dl_config.h"
#include "motor.h"
#include "stdio.h"
#include <stdint.h>
#include "stdbool.h"
extern uint8_t HD[8];
extern int16_t SpeedL;
extern int16_t SpeedR;
extern uint8_t beep_flag;
extern float x,y,z;
extern float z1;//程序上电读初始偏航角
extern float z1temp;
extern float zACtemp;//程序3调用
extern float zBDtemp;//程序3调用
extern uint8_t flag;//不同的子测试程序
extern uint8_t runflag;
extern uint8_t beep_flag_num;//路过标记的总计数，用这个参数推断圈数
extern char str[50];
extern int16_t Roll;//俯仰角
extern int16_t Pitch;//滚转角
extern int16_t Yaw;//偏航角
extern float x,y,z;//陀螺仪返回程序计算值
extern uint8_t ACflag;
extern uint8_t BDflag;
#define BUFFER_SIZE 10
extern uint8_t rxBuffer[BUFFER_SIZE];
extern uint8_t turnLflag;
extern uint8_t turnRflag;
void HD_Read(void);//读取灰度值
void HD_MOTO(void);//巡线半圆
void HD_MOTO1(void);//1题
float absfloat(float a);
void HD_MOTO2(void);//2题
void beep_ctrl(void);//控制蜂鸣器响
void runandrun(float ztest);//笔直向前跑
void oled_show(void);//OLED刷新数据
void uart_deal(void);//陀螺仪串口数据处理
void moto_off(void);//关闭电机
void HD_MOTO0(void);//测试程序
int16_t Location_Pid_PWM(float set_angle, float current_angle);//位置式PID
void goAtoC(void);//AC转向
void goBtoD(void);//AC转向
void HD_MOTO3(void);//3题
float absz(float a,float b);//偏航轴差计算
void HD_MOTO4(void);
void HD_MOTO_T3(void);

#endif

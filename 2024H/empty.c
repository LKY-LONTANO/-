/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "ti_msp_dl_config.h"
#include "motor.h"
#include "oled.h"
#include "stdio.h"
#include <stdint.h>
#include "stdbool.h"
#include "hd.h"
#include "empty.h"

int16_t baseSpeed = 0;//测试用函数，测试电机转动的基准速度
int16_t SpeedL = 0, SpeedR = 0;//程序使用，左右轮转速，值为高电平占空比
char str[50];
float a=0.0;
int b=0;
uint8_t HD[8];//存放8路灰度数据
volatile unsigned char uart_data = 0;//用于串口接收的变量
uint8_t buff_zhizhen=0;//指针，用于串口接收的字节增加地址
float x,y,z;//存储陀螺仪三个轴的数据，注意程序只用到了Z轴偏航角，x、y在这里是多余的定义
float z1;//用来存储小车上电后执行程序存储的起始偏航角数据，注意该角度十分重要，小车后续所有转向的角度全靠该数据进行计算
float z1temp;//存储C->D的角度
float zACtemp;//绋嬪簭3璋冪敤
float zBDtemp;//绋嬪簭3璋冪敤
uint8_t flag=5;//不同的flag用于后续执行不同的程序，当前程序flag由上电后按钮选择，详见后续代码
uint8_t beep_flag=0;//标识蜂鸣器鸣叫的标记位
uint8_t beep_flag_num=0;//蜂鸣器鸣叫的次数统计，注意，可以根据该值来判断小车处于第几圈的什么位置，用于编写有关的代码
uint8_t runflag=0;//该参数用来标识小车实在圆弧轨迹上面还是白地上面
int16_t Roll;//用于存储x轴原始陀螺仪读数，是个中间变量
int16_t Pitch;//用于存储x轴原始陀螺仪读数，是个中间变量
int16_t Yaw;//用于存储x轴原始陀螺仪读数，是个中间变量
uint8_t ACflag=0;//标识小车走AC线还是BD线的标记，根据该标记确定小车出圆弧区后是左转还是右转
uint8_t BDflag=0;//标识小车走AC线还是BD线的标记，根据该标记确定小车出圆弧区后是左转还是右转
uint8_t turnLflag=0;//小车在走AC、BD线进圆弧区时会进行强制转向（到指定通道的灰度返回1时结束转向），这个标记用于标记转向方向
uint8_t turnRflag=0;//小车在走AC、BD线进圆弧区时会进行强制转向（到指定通道的灰度扫到黑线时结束转向），这个标记用于标记转向方向

//用来存放陀螺仪接收到的数据
uint8_t rxBuffer[BUFFER_SIZE];

void Motor_Ctrl(void);//测试电机转动
void uart2_send_char(char ch);//串口2发送字节
void uart2_send_string(char* str);//串口2发送字符串
void UART_WIT_INST_IRQHandler(void);//串口2接收中断
void uart_deal(void);//串口2数据处理

int main(void)
{
    SYSCFG_DL_init();
    //系统初始化
    NVIC_ClearPendingIRQ(UART_WIT_INST_INT_IRQN);
    //初始化串口中断
    NVIC_EnableIRQ(UART_WIT_INST_INT_IRQN);
    //OLED初始化
    OLED_Init();
    OLED_ColorTurn(0);
    OLED_DisplayTurn(0);
    OLED_Clear();

    //while(DL_GPIO_readPins(GPIO_BUTTON_PORT, GPIO_BUTTON_PIN_S2_PIN));
    while(1){//通过读取GPIO引脚电平来判断四个按键的按下状态，确定执行哪个程序
        if(DL_GPIO_readPins(GPIO_BUTTON_K1_PORT, GPIO_BUTTON_K1_PIN)==0){flag=1;break;}
        if(DL_GPIO_readPins(GPIO_BUTTON_K2_PORT, GPIO_BUTTON_K2_PIN)==0){flag=2;break;}
        if(DL_GPIO_readPins(GPIO_BUTTON_K3_PORT, GPIO_BUTTON_K3_PIN)==0){flag=3;break;}
        if(DL_GPIO_readPins(GPIO_BUTTON_K4_PORT, GPIO_BUTTON_K4_PIN)==0){flag=4;break;}
    }
//    DL_UART_transmitDataBlocking(UART_OPENMV_INST, 'S');

    delay_cycles(32000000);//时钟初始化，用于delay_ms

    Set_Speed(0, baseSpeed);//设置左右轮速，初始不动
    Set_Speed(1, baseSpeed);
    DL_GPIO_clearPins(GPIO_LED_PORT,GPIO_LED_PIN_LED1_PIN);//点亮LED
    delay_ms(1000);//等待陀螺仪初始化和串口中断接收完起始数据
    uart_deal();//串口数据处理，获取起始偏航角z
    DL_GPIO_setPins(GPIO_LED_PORT,GPIO_LED_PIN_LED1_PIN);//关闭LED
    z1=z;z1temp=z1- 180;//z是起始偏航角，也是A->B的方向；z1temp是C->D的方向，与z反向
    if (z1temp < -180)//数据处理，类似的程序能够保证计算出来的角度是绝对角度，由陀螺仪处理得到的角度范围是-180~180。
        {
            z1temp = z1temp + 360;
        }
        else if (z1temp > 180)
        {
            z1temp = z1temp - 360;
        }
    zACtemp=z1 - 40;//计算A->C线的角度,小车右转，绝对角度减小
    if (zACtemp < -180)
        {
            zACtemp = zACtemp + 360;
        }
    else if (zACtemp > 180)
        {
            zACtemp = zACtemp - 360;
    }
    zBDtemp=z1temp + 40;//计算B->D线的角度,小车左转，绝对角度增大
    if (zBDtemp < -180)
        {
            zBDtemp = zBDtemp + 360;
        }
    else if (zBDtemp > 180)
        {
            zBDtemp = zBDtemp - 360;
    }
  while (1)
    {
        OLED_Refresh();//屏幕数据刷新
        HD_Read();//获取灰度数据
        uart_deal();//更新陀螺仪数据
        switch (flag) {
            case 0:
                HD_MOTO0();//测试程序，调试使用
                //HD_MOTO();
                break;
            case 1:
                HD_MOTO1();//第一题的程序
                break;
            case 2:
                HD_MOTO2();//第二题的程序
                break;
            case 3:
                HD_MOTO3();//第三题的程序
                break;
            case 4:
                HD_MOTO4();//第四题的程序
                break;
            default:
                break;
        }
    }

}

void Motor_Ctrl(void)
{
    SpeedL = baseSpeed;
    SpeedR = baseSpeed;

    if(SpeedL > 30)
        SpeedL = 30;
    else if(SpeedL < 0)
        SpeedL = 0;

    if(SpeedR > 30)
        SpeedR = 30;
    else if(SpeedR < 0)
        SpeedR = 0;

    Set_Speed(0, SpeedL);
    Set_Speed(1, SpeedR);
}

//串口数据（字节）发送，源码借鉴自立创开发板
void uart2_send_char(char ch)
{
    while( DL_UART_isBusy(UART_WIT_INST) == true );
    DL_UART_Main_transmitData(UART_WIT_INST, ch);
}
//串口数据（字符串）发送，源码借鉴自立创开发板
void uart2_send_string(char* str)
{
    while(*str!=0&&str!=0)
    {
        uart2_send_char(*str++);
    }
}

//串口2中断，陀螺仪
void UART_WIT_INST_IRQHandler(void)
{
    switch( DL_UART_getPendingInterrupt(UART_WIT_INST) )
    {
        case DL_UART_IIDX_RX:
            uart_data =DL_UART_Main_receiveData(UART_WIT_INST);
            if(uart_data==0x55)buff_zhizhen=0;
            rxBuffer[buff_zhizhen]=uart_data;
            buff_zhizhen++;
            if(buff_zhizhen==BUFFER_SIZE){buff_zhizhen=0;}
            break;

        default:
            break;
    }
}

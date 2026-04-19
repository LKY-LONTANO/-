#include "bluetooth.h"
#include "balance.h"

#include <string.h>

#include "bsp_printf.h"
#include "bsp_systick.h"

#define BT_PACKET_SIZE (200)
volatile uint8_t gBTPacket[BT_PACKET_SIZE];
volatile uint8_t gBTCounts = 0 ;
volatile uint8_t lastBTCounts = 0 ;

//是否需要发送PID参数
uint8_t PID_Send = 0;

void BT_DAMConfig(void)
{
    DL_DMA_disableChannel(DMA, DMA_CH0_CHAN_ID);
    DL_DMA_setSrcAddr(DMA, DMA_CH0_CHAN_ID, (uint32_t)(&UART_1_INST->RXDATA));
    DL_DMA_setDestAddr(DMA, DMA_CH0_CHAN_ID, (uint32_t) &gBTPacket[0]);
    DL_DMA_setTransferSize(DMA, DMA_CH0_CHAN_ID, BT_PACKET_SIZE);
    DL_DMA_enableChannel(DMA, DMA_CH0_CHAN_ID);
}

//用于轮询检查dma搬运后是否空闲,实现类似于STM32使用DMA+空闲中断的效果
void BTBufferHandler(void)
{
    uint32_t tick = 0;
    static uint8_t handleflag = 0;
    static uint8_t handleSize = 0;
    static uint8_t lastSize = 0;

    //recvsize为已经接收的数据个数
    uint8_t recvsize = BT_PACKET_SIZE - DL_DMA_getTransferSize(DMA, DMA_CH0_CHAN_ID);

    uint32_t ticks = 0;
    if( recvsize != lastSize)
    {
        handleflag=1;
        tick = Systick_getTick();//正在接收数据,刷新时间
    }
    else
    {
        //数据已停止接收,超时 1ms 后开始处理数据
        if( ((tick-Systick_getTick())&SysTickMAX_COUNT) >= SysTick_MS(1) && handleflag == 1)
        {
            handleflag = 0;

            //处理数据
            // for(uint8_t i=handleSize;i<recvsize;i++)
            //     printf("%c",gBTPacket[i]);
            // printf("\r\n");

            //处理串口数据
            for(uint8_t i=handleSize;i<recvsize;i++)
                bt_control(gBTPacket[i]);


            //记录本次数据处理的位置
            handleSize = recvsize;

            //处理到一定数据量时刷新一次dma搬运地址,不让dma完成搬运
            if( recvsize >= BT_PACKET_SIZE/2 )
            {
                recvsize=0;
                handleSize=0;
                lastSize = 0;
                BT_DAMConfig();
            }
        }
    }

    //执行处理结果
    //检查是否需要发送参数到APP
    if(PID_Send==1)
    {
        any_printf(UART_1_INST,"{C%d:%d:%d:%d:%d:%d:%d:%d:%d}$",
        (int)Balance_Kp,(int)Balance_Kd,(int)Velocity_Kp,(int)Velocity_Ki,(int)Turn_Kp,(int)Turn_Kd,0,0,0);//打印到APP上面
        PID_Send=0;
    }

    lastSize = recvsize;
}

//蓝牙APP数据处理
void bt_control(uint8_t recv)
{
    static  int uart_receive=0;//蓝牙接收相关变量
    static uint8_t Flag_PID,i,j,Receive[50];
    static float Data;

    // 接收发送过来的数据保存
    uart_receive = recv;

    if( uart_receive == 'a' ) Flag_Stop=!Flag_Stop;

    if(uart_receive>10)  //默认使用
    {
            if(uart_receive==0x5A)      Flag_front=0,Flag_back=0,Flag_Left=0,Flag_Right=0;//刹车
            else if(uart_receive==0x41) Flag_front=1,Flag_back=0,Flag_Left=0,Flag_Right=0;//前
            else if(uart_receive==0x45) Flag_front=0,Flag_back=1,Flag_Left=0,Flag_Right=0;//后
            else if(uart_receive==0x42||uart_receive==0x43||uart_receive==0x44)
                                                                    Flag_front=0,Flag_back=0,Flag_Left=0,Flag_Right=1;  //右
            else if(uart_receive==0x46||uart_receive==0x47||uart_receive==0x48)
                                                                    Flag_front=0,Flag_back=0,Flag_Left=1,Flag_Right=0;  //左
            else Flag_front=0,Flag_back=0,Flag_Left=0,Flag_Right=0;//刹车
    }

    if(uart_receive==0x7B) Flag_PID=1;   //APP参数指令起始位
    if(uart_receive==0x7D) Flag_PID=2;   //APP参数指令停止位
    if(Flag_PID==1)  //采集数据
    {
            Receive[i]=uart_receive;
            i++;
    }
    if(Flag_PID==2)  //分析数据
    {
            if(Receive[3]==0x50)               PID_Send=1;
            else if(Receive[1]!=0x23)
            {
                for(j=i;j>=4;j--)
                {
                    Data+=(Receive[j-1]-48)*pow(10,i-j);
                }
                switch(Receive[1])
                {
                    case 0x30:  Balance_Kp=Data;break;
                    case 0x31:  Balance_Kd=Data;break;
                    case 0x32:  Velocity_Kp=Data;break;
                    case 0x33:  Velocity_Ki=Data;break;
                    case 0x34:  Turn_Kp=Data;break;
                    case 0x35:  Turn_Kd=Data;break;
                    case 0x36:  break; //预留
                    case 0x37:  break; //预留
                    case 0x38:  break; //预留
                }
            }
                Flag_PID=0;
                i=0;
                j=0;
                Data=0;
                memset(Receive, 0, sizeof(uint8_t)*50);//数组清零
    }

}

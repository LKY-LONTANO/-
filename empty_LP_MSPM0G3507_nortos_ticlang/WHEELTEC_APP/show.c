#include "show.h"
#include "bsp_oled.h"

//OLED接口
static pOLEDInterface_t oled = &UserOLED;

//平衡任务的频率
extern uint8_t mainTaskFreq;

//平衡任务耗时
extern float mainTaskUseTime;

void oled_show(void)
{
    //第一行显示平衡任务的频率
    oled->ShowString(0,0,"fre:");
    oled->ShowNumber(45,0,mainTaskFreq,3,12);
    
    //显示平衡任务的总体耗时,单位ms
    oled->ShowFloat(80,0,mainTaskUseTime,2,2);

    //第二行显示角度
    oled->ShowString(0,10,"angle:");
    oled->ShowFloat(50,10,mpu6050.roll,3,2);

    //第三行显示x
    oled->ShowString(0,20,"gyrox:");
    oled->ShowFloat(50,20,mpu6050.gyro.x,3,2);

    //第四、五行显示车轮信息
    oled->ShowString(0,30,"L");
    if( Motor_Left < 0 )  oled->ShowString(16,30,"-");
    else oled->ShowString(16,30,"+");
    oled->ShowNumber(26,30,abs(Motor_Left),4,12);

    if( Velocity_Left < 0 )  oled->ShowString(60,30,"-");
    else oled->ShowString(60,30,"+");
    oled->ShowNumber(68,30,abs((int)Velocity_Left),4,12);
    oled->ShowString(96,30,"mm/s");

    oled->ShowString(0,40,"R");
    if( Motor_Left < 0 )  oled->ShowString(16,40,"-");
    else oled->ShowString(16,40,"+");
    oled->ShowNumber(26,40,abs(Motor_Right),4,12);

    if( Velocity_Left < 0 )  oled->ShowString(60,40,"-");
    else oled->ShowString(60,40,"+");
    oled->ShowNumber(68,40,abs((int)Velocity_Right),4,12);
    oled->ShowString(96,40,"mm/s");

    //第六行显示电压、使能情况
    oled->ShowFloat(0,50,robotVol,2,2);
    oled->ShowString(50,50,"V");

    if( Flag_Stop )
        oled->ShowString(96,50,"OFF");
    else
        oled->ShowString(96,50," ON");

    //刷新显示
    oled->RefreshGram();
}

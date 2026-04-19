#include "hd.h"
#include "i2c.h"
#include "oled.h"
#include "empty.h"
//弧线灰度寻线



#define Position_KP 19.2 //需要整定的PID参数
#define Position_KI 0.8   //0.00004
#define Position_KD 2.8

void HD_Read(void){//灰度数据获取，在硬件上使用8路数字量灰度模块，当某一路扫到黑线时，将返回高电平
    if(DL_GPIO_readPins(GPIO_HD_AD0_PORT, GPIO_HD_AD0_PIN)==0){HD[0]=0;}else{HD[0]=1;}
    if(DL_GPIO_readPins(GPIO_HD_AD1_PORT, GPIO_HD_AD1_PIN)==0){HD[1]=0;}else{HD[1]=1;}
    if(DL_GPIO_readPins(GPIO_HD_AD2_PORT, GPIO_HD_AD2_PIN)==0){HD[2]=0;}else{HD[2]=1;}
    if(DL_GPIO_readPins(GPIO_HD_AD3_PORT, GPIO_HD_AD3_PIN)==0){HD[3]=0;}else{HD[3]=1;}
    if(DL_GPIO_readPins(GPIO_HD_AD4_PORT, GPIO_HD_AD4_PIN)==0){HD[4]=0;}else{HD[4]=1;}
    if(DL_GPIO_readPins(GPIO_HD_AD5_PORT, GPIO_HD_AD5_PIN)==0){HD[5]=0;}else{HD[5]=1;}
    if(DL_GPIO_readPins(GPIO_HD_AD6_PORT, GPIO_HD_AD6_PIN)==0){HD[6]=0;}else{HD[6]=1;}
    if(DL_GPIO_readPins(GPIO_HD_AD7_PORT, GPIO_HD_AD7_PIN)==0){HD[7]=0;}else{HD[7]=1;}
}


//圆弧巡线程序
void HD_MOTO(void){//通过灰度数据来调整左右两侧轮子的差速实现转向，以小车车头为正方向，8路数据的HD下标为0-7
    HD_Read();
    if(HD[0]==1 && HD[1]==1 && HD[2]==0 && HD[3]==0 && HD[4]==0 && HD[5]==0 && HD[6]==0 && HD[7]==0)
    {
        SpeedL=-40;
        SpeedR=50;
        Set_Speed(0, SpeedL);
        Set_Speed(1, SpeedR);
        delay_ms(15);
    }else if(HD[0]==1 && HD[1]==0 && HD[2]==0 && HD[3]==0 && HD[4]==0 && HD[5]==0 && HD[6]==0 && HD[7]==0)
    {
        SpeedL=-40;
        SpeedR=50;
        Set_Speed(0, SpeedL);
        Set_Speed(1, SpeedR);
        delay_ms(15);
    }else if(HD[0]==0 && HD[1]==1 && HD[2]==0 && HD[3]==0 && HD[4]==0 && HD[5]==0 && HD[6]==0 && HD[7]==0)
    {
        SpeedL=-40;
        SpeedR=50;
        Set_Speed(0, SpeedL);
        Set_Speed(1, SpeedR);
        delay_ms(15);
    }else if(HD[0]==0 && HD[1]==0 && HD[2]==1 && HD[3]==1 && HD[4]==0 && HD[5]==0 && HD[6]==0 && HD[7]==0)
    {
        SpeedL=-10;
        SpeedR=50;
        Set_Speed(0, SpeedL);
        Set_Speed(1, SpeedR);
        delay_ms(10);
    }else if(HD[0]==0 && HD[1]==0 && HD[2]==1 && HD[3]==0 && HD[4]==0 && HD[5]==0 && HD[6]==0 && HD[7]==0)
    {
        SpeedL=-10;
        SpeedR=50;
        Set_Speed(0, SpeedL);
        Set_Speed(1, SpeedR);
        delay_ms(10);
    }else if(HD[0]==0 && HD[1]==0 && HD[2]==0 && HD[3]==1 && HD[4]==0 && HD[5]==0 && HD[6]==0 && HD[7]==0)
    {
        SpeedL=10;
        SpeedR=30;
        Set_Speed(0, SpeedL);
        Set_Speed(1, SpeedR);
        delay_ms(5);
    }else if(HD[0]==0 && HD[1]==0 && HD[2]==0 && HD[3]==1 && HD[4]==1 && HD[5]==0 && HD[6]==0 && HD[7]==0)
    {
        SpeedL=20;
        SpeedR=20;
        Set_Speed(0, SpeedL);
        Set_Speed(1, SpeedR);
        //delay_ms(1);
    }else if(HD[0]==0 && HD[1]==0 && HD[2]==0 && HD[3]==0 && HD[4]==1 && HD[5]==0 && HD[6]==0 && HD[7]==0)
    {
        SpeedL=30;
        SpeedR=10;
        Set_Speed(0, SpeedL);
        Set_Speed(1, SpeedR);
        delay_ms(5);
    }else if(HD[0]==0 && HD[1]==0 && HD[2]==0 && HD[3]==0 && HD[4]==0 && HD[5]==1 && HD[6]==0 && HD[7]==0)
    {
        SpeedL=50;
        SpeedR=-10;
        Set_Speed(0, SpeedL);
        Set_Speed(1, SpeedR);
        delay_ms(10);
    }else if(HD[0]==0 && HD[1]==0 && HD[2]==0 && HD[3]==0 && HD[4]==1 && HD[5]==1 && HD[6]==0 && HD[7]==0)
    {
        SpeedL=50;
        SpeedR=-10;
        Set_Speed(0, SpeedL);
        Set_Speed(1, SpeedR);
        delay_ms(10);
    }else if(HD[0]==0 && HD[1]==0 && HD[2]==0 && HD[3]==0 && HD[4]==0 && HD[5]==0 && HD[6]==1 && HD[7]==0)
    {
        SpeedL=50;
        SpeedR=-40;
        Set_Speed(0, SpeedL);
        Set_Speed(1, SpeedR);
        delay_ms(15);
    }else if(HD[0]==0 && HD[1]==0 && HD[2]==0 && HD[3]==0 && HD[4]==0 && HD[5]==0 && HD[6]==0 && HD[7]==1)
    {
        SpeedL=50;
        SpeedR=-40;
        Set_Speed(0, SpeedL);
        Set_Speed(1, SpeedR);
        delay_ms(15);
    }else if(HD[0]==0 && HD[1]==0 && HD[2]==0 && HD[3]==0 && HD[4]==0 && HD[5]==0 && HD[6]==1 && HD[7]==1)
    {
        SpeedL=50;
        SpeedR=-40;
        Set_Speed(0, SpeedL);
        Set_Speed(1, SpeedR);
        delay_ms(15);
    }else if(HD[0]==0 && HD[1]==0 && HD[2]==0 && HD[3]==0 && HD[4]==0 && HD[5]==1 && HD[6]==1 && HD[7]==0)
    {
        SpeedL=50;
        SpeedR=-10;
        Set_Speed(0, SpeedL);
        Set_Speed(1, SpeedR);
        delay_ms(10);
    }
    else if(HD[0]==0 && HD[1]==1 && HD[2]==1 && HD[3]==0 && HD[4]==0 && HD[5]==0 && HD[6]==0 && HD[7]==0)
    {
        SpeedL=-10;
        SpeedR=50;
        Set_Speed(0, SpeedL);
        Set_Speed(1, SpeedR);
        delay_ms(10);
    }
    else if(HD[0]==1 && HD[1]==1 && HD[2]==1)
    {
        SpeedL=50;
        SpeedR=-10;
        Set_Speed(0, SpeedL);
        Set_Speed(1, SpeedR);
        delay_ms(10);
    }
    else if(HD[1]==1 && HD[2]==1 && HD[3]==1)
    {
        SpeedL=-10;
        SpeedR=50;
        Set_Speed(0, SpeedL);
        Set_Speed(1, SpeedR);
        delay_ms(10);
    }
    else if(HD[5]==1 && HD[6]==1 && HD[7]==1)
    {
        SpeedL=50;
        SpeedR=-10;
        Set_Speed(0, SpeedL);
        Set_Speed(1, SpeedR);
        delay_ms(10);
    }
    else if(HD[4]==1 && HD[5]==1 && HD[6]==1)
    {
        SpeedL=50;
        SpeedR=-10;
        Set_Speed(0, SpeedL);
        Set_Speed(1, SpeedR);
        delay_ms(10);
    }
    else if(HD[0]==0 && HD[1]==0 && HD[2]==0 && HD[3]==0 && HD[4]==0 && HD[5]==0 && HD[6]==0 && HD[7]==0)
    {//当小车扫不到黑线时，程序控制小车延时前移一段，仍然扫不到线的情况下，且小车偏航角与AB、CD方向夹角小于18°，认定为小车离开圆弧区
        SpeedL=15;
        SpeedR=15;
        Set_Speed(0, SpeedL);
        Set_Speed(1, SpeedR);
        delay_ms(10);
        HD_Read();
        if(HD[0]==0 && HD[1]==0 && HD[2]==0 && HD[3]==0 && HD[4]==0 && HD[5]==0 && HD[6]==0 && HD[7]==0){
            if((absfloat(absz(z1temp,z))<18)){
                runflag=0;
            }else if((absfloat(absz(z1,z))<18)){
                runflag=0;//标记进入白地，外部程序会根据该标记进行声光提示
            }
        }
    }
    else{
        SpeedL=-20;
        SpeedR=20;
        Set_Speed(0, SpeedL);
        Set_Speed(1, SpeedR);
    }
}

void HD_MOTO1(void)
{
    if(HD[0]==0 && HD[1]==0 && HD[2]==0 && HD[3]==0 && HD[4]==0 && HD[5]==0 && HD[6]==0 && HD[7]==0)
    {
            uart_deal();
            runandrun(z1);//按z1角度直线行驶     
            HD_Read();//读取灰度数据
    }
    if(HD[0]==1 || HD[1]==1 || HD[2]==1 || HD[3]==1 || HD[4]==1 || HD[5]==1 || HD[6]==1 || HD[7]==1)
    {
            SpeedL=0;
            SpeedR=0;
            Set_Speed(0, SpeedL);
            Set_Speed(1, SpeedR);
            delay_ms(20);
            beep_flag=1;
    }
    if(beep_flag==1){//声光提示
        DL_GPIO_clearPins(GPIO_BEELIGHT_PORT, GPIO_BEELIGHT_BEEP_PIN);
        DL_GPIO_setPins(GPIO_BEELIGHT_PORT, GPIO_BEELIGHT_LIGHT_PIN);
        delay_ms(1000);
        DL_GPIO_setPins(GPIO_BEELIGHT_PORT, GPIO_BEELIGHT_BEEP_PIN);
        DL_GPIO_clearPins(GPIO_BEELIGHT_PORT, GPIO_BEELIGHT_LIGHT_PIN);
        beep_flag=0;
        flag=5;//程序退出
    }
}

void HD_MOTO2(void)
{
    if(HD[0]==0 && HD[1]==0 && HD[2]==0 && HD[3]==0 && HD[4]==0 && HD[5]==0 && HD[6]==0 && HD[7]==0 && runflag==0){//鍦ㄧ櫧鍦颁笂闈㈢洿璺�
       if(beep_flag_num==0){
           uart_deal();
           runandrun(z1-2);//按z1角度直线行驶，A->B,并对陀螺仪漂移的问题进行修正
           HD_Read();
           oled_show();
       }
       if(beep_flag_num==2){
           uart_deal();
           runandrun(z1temp-2);//按z1temp角度直线行驶，C->D，并对陀螺仪漂移的问题进行修正
           HD_Read();
           oled_show();
       }
    }
    if(HD[0]==1 || HD[1]==1 || HD[2]==1 || HD[3]==1 || HD[4]==1 || HD[5]==1 || HD[6]==1 || HD[7]==1 && runflag==0){//扫到黑线，且之前处于白地
        SpeedL=10;
        SpeedR=10;
        Set_Speed(0, SpeedL);
        Set_Speed(1, SpeedR);
        delay_ms(3);//由于给定的PWM太小，上面的几句程序几乎不起作用，视跑车情况进行更改
        if(HD[0]==1 || HD[1]==1 || HD[2]==1 || HD[3]==1 || HD[4]==1 || HD[5]==1 || HD[6]==1 || HD[7]==1 && runflag==0){//按一定速度前移一段，仍然扫到线判定为进入圆弧区
            runflag=1;
            beep_flag=1;
            moto_off();
            beep_ctrl();//声光提示
            beep_flag_num++;//增加记录的蜂鸣器鸣叫次数，每次鸣叫都是一个节点
            while(runflag){
                HD_MOTO();//巡线程序
                uart_deal();
                oled_show();}
            if(runflag==0){
                beep_flag=1;
                moto_off();
                beep_ctrl();//声光提示
                beep_flag_num++;
            }
       }
    }
    if(beep_flag_num==4)flag=5;//程序退出
}

void HD_MOTO3(void)
{
    if(HD[0]==0 && HD[1]==0 && HD[2]==0 && HD[3]==0 && HD[4]==0 && HD[5]==0 && HD[6]==0 && HD[7]==0 && runflag==0){
        if(beep_flag_num==0){
              uart_deal();
              goAtoC();//转向
              runandrun(zACtemp+1.5);//按AC角度直线行驶，A->C，并对陀螺仪漂移的问题进行修正
              turnLflag=1;
              //turnRflag=0;
              HD_Read();
              oled_show();
        }
        if(beep_flag_num==2){
              uart_deal();
              goBtoD();//转向
              runandrun(zBDtemp-3.2);//按BD角度直线行驶，B->D，并对陀螺仪漂移的问题进行修正
              turnRflag=1;
              //turnLflag=0;
              HD_Read();
              oled_show();
        }
    }
    if(HD[0]==1 || HD[1]==1 || HD[2]==1 || HD[3]==1 || HD[4]==1 || HD[5]==1 || HD[6]==1 || HD[7]==1 && runflag==0){
        SpeedL=10;
        SpeedR=10;
        Set_Speed(0, SpeedL);
        Set_Speed(1, SpeedR);
        delay_ms(3);
        HD_Read();
        if(HD[0]==1 || HD[1]==1 || HD[2]==1 || HD[3]==1 || HD[4]==1 || HD[5]==1 || HD[6]==1 || HD[7]==1 && runflag==0){
            ACflag=0;//由白地扫线进入黑地
            BDflag=0;
            runflag=1;
            beep_flag=1;
            moto_off();//停车
            beep_ctrl();//鸣叫
            beep_flag_num++;
            if(turnLflag==1){//根据节点选择强制左转还是右转以避免小车冲线导致的踩不到黑线
                while(HD[0]==0 || HD[1]==0 || HD[2]==0){
                    HD_Read();
                    if(HD[5]==1)break;
                    SpeedL=-40;
                    SpeedR=50;
                    Set_Speed(0, SpeedL);
                    Set_Speed(1, SpeedR);
                }
                turnLflag=0;
            }
            if(turnRflag==1){
                while(HD[5]==0 || HD[6]==0 || HD[7]==0){
                    HD_Read();
                    if(HD[2]==1)break;
                    SpeedL=50;
                    SpeedR=-40;
                    Set_Speed(0, SpeedL);
                    Set_Speed(1, SpeedR);
                }
                turnRflag=0;
            }
            while(runflag){//循环寻迹，来到白地时跳出
                HD_MOTO_T3();//给第三题单独写的小车寻迹程序，实则是为了调试第四题时，不影响第三题已经实现的小车效果做出的选择
                uart_deal();//偏航角更新
                oled_show();}
            if(runflag==0){
                beep_flag=1;
                moto_off();//停车
                beep_ctrl();//鸣叫
                beep_flag_num++;//增加节点
            }
       }
    }
    if(beep_flag_num==4)flag=5;//程序退出
}

void HD_MOTO4(void)
{
    if(HD[0]==0 && HD[1]==0 && HD[2]==0 && HD[3]==0 && HD[4]==0 && HD[5]==0 && HD[6]==0 && HD[7]==0 && runflag==0){
        //第四题程序与第三题程序逻辑几乎完全相同，只不过第四题小车需要跑四圈，节点更多，小车陀螺仪偏移在后面两圈非常明显，因此，根据蜂鸣器鸣叫的次数做节点标记，调整小车转向的偏移
       if(beep_flag_num==0){
           uart_deal();//更新偏航角
           goAtoC();//转向
           runandrun(zACtemp+1.5);//直线行驶
           turnLflag=1;//标记左转标记，后续程序根据该标记在小车进入圆弧区时进行强制的左转向
           //turnRflag=0;
           HD_Read();//更新灰度数据
           oled_show();//更新OLED存储，OLED刷屏函数在外部上级函数中
       }
       if(beep_flag_num==2){
           uart_deal();
           goBtoD();
           runandrun(zBDtemp-3.2);
           turnRflag=1;
           //turnLflag=0;
           HD_Read();
           oled_show();
       }
       if(beep_flag_num==4){
           uart_deal();
           goAtoC();
           runandrun(zACtemp+2.5);
           turnLflag=1;
           //turnRflag=0;
           HD_Read();
           oled_show();
       }
       if(beep_flag_num==6){
           uart_deal();
           goBtoD();
           runandrun(zBDtemp-3.6);
           turnRflag=1;
           //turnLflag=0;
           HD_Read();
           oled_show();
       }
       if(beep_flag_num==8){
           uart_deal();
           goAtoC();
           runandrun(zACtemp+2.8);
           turnLflag=1;
           //turnRflag=0;
           HD_Read();
           oled_show();
       }
       if(beep_flag_num==10){
           uart_deal();
           goBtoD();
           runandrun(zBDtemp-3.6);
           turnRflag=1;
           //turnLflag=0;
           HD_Read();
           oled_show();
       }
       if(beep_flag_num==12){
           uart_deal();
           goAtoC();
           runandrun(zACtemp+3.0);
           turnLflag=1;
           //turnRflag=0;
           HD_Read();
           oled_show();
       }
       if(beep_flag_num==14){
           uart_deal();
           goBtoD();
           runandrun(zBDtemp-4.0);
           turnRflag=1;
           //turnLflag=0;
           HD_Read();
           oled_show();
       }
    }
    if(HD[0]==1 || HD[1]==1 || HD[2]==1 || HD[3]==1 || HD[4]==1 || HD[5]==1 || HD[6]==1 || HD[7]==1 && runflag==0){
        if(HD[0]==1 || HD[1]==1 || HD[2]==1 || HD[3]==1 || HD[4]==1 || HD[5]==1 || HD[6]==1 || HD[7]==1 && runflag==0){
            ACflag=0;//似乎是多余定义的变量
            BDflag=0;
            runflag=1;
            beep_flag=1;
            moto_off();
            beep_ctrl();
            beep_flag_num++;
            if(turnLflag==1){
                while(HD[0]==0 || HD[1]==0 || HD[2]==0){
                    HD_Read();
                    if(HD[5]==1)break;
                    SpeedL=-40;
                    SpeedR=50;
                    Set_Speed(0, SpeedL);
                    Set_Speed(1, SpeedR);
                }
                turnLflag=0;
            }
            if(turnRflag==1){
                while(HD[5]==0 || HD[6]==0 || HD[7]==0){
                    HD_Read();
                    if(HD[2]==1)break;
                    SpeedL=50;
                    SpeedR=-40;
                    Set_Speed(0, SpeedL);
                    Set_Speed(1, SpeedR);
                }
                turnRflag=0;
            }
            while(runflag){
                HD_MOTO();
                uart_deal();
                }
            if(runflag==0){
                beep_flag=1;
                moto_off();
                beep_ctrl();
                beep_flag_num++;
            }
       }
    }
    if(beep_flag_num==16)flag=5;//程序退出
}

float absfloat(float a)
{
    return (a >= 0) ? a : -a;
}

void beep_ctrl(void){
    if(beep_flag==1){
            DL_GPIO_clearPins(GPIO_BEELIGHT_PORT, GPIO_BEELIGHT_BEEP_PIN);
            DL_GPIO_setPins(GPIO_BEELIGHT_PORT, GPIO_BEELIGHT_LIGHT_PIN);
            delay_ms(500);
            DL_GPIO_setPins(GPIO_BEELIGHT_PORT, GPIO_BEELIGHT_BEEP_PIN);
            DL_GPIO_clearPins(GPIO_BEELIGHT_PORT, GPIO_BEELIGHT_LIGHT_PIN);
            beep_flag=0;//清零声光提示标记位
    }
}

void runandrun(float ztest)//按指定的绝对角度进行直线行驶
{
    SpeedL=20-Location_Pid_PWM(ztest,z);
    SpeedR=20+Location_Pid_PWM(ztest,z);
    Set_Speed(0, SpeedL);
    Set_Speed(1, SpeedR);
}

void oled_show(void){//OLED显示
    sprintf(str,"1:%d 2:%d 3:%d 4:%d",HD[0],HD[1],HD[2],HD[3]);
    OLED_ShowString(0,0,(u8 *)str,16,1);
    sprintf(str,"5:%d 6:%d 7:%d 8:%d",HD[4],HD[5],HD[6],HD[7]);
    OLED_ShowString(0,16,(u8 *)str,16,1);
    sprintf(str,"z1:%.2f  zA:%.2f   ",z1temp,zBDtemp);
    OLED_ShowString(0,32,(u8 *)str,8,1);
    sprintf(str,"z:%.2f     ",z);
    OLED_ShowString(0,48,(u8 *)str,16,1);
}

void uart_deal(void){//更新陀螺仪数据
        if (rxBuffer[0] == 0x55 && rxBuffer[1] == 0x53) {
            Roll = (int16_t)((rxBuffer[3] << 8) | rxBuffer[2]);
            Pitch = (int16_t)((rxBuffer[5] << 8) | rxBuffer[4]);
            Yaw = (int16_t)((rxBuffer[7] << 8) | rxBuffer[6]);
            x=(float)Roll/32768*180;
            y=(float)Pitch/32768*180;
            z=(float)Yaw/32768*180;
        }
}

void moto_off(){//停车
    SpeedL=0;
    SpeedR=0;
    Set_Speed(0, SpeedL);
    Set_Speed(1, SpeedR);
}

void HD_MOTO0(){//测试程序
    uart_deal();
    HD_Read();
    oled_show();
    //SpeedL=-Location_Pid_PWM(zACtemp,z);
    //SpeedR=Location_Pid_PWM(zACtemp,z);
    //Set_Speed(0, SpeedL);
    //Set_Speed(1, SpeedR);
}

/*位置式PID
*float set_angle:设定的目标角度
current_angle:当前的角度
返回输出目标占空比，左轮占空比-输出值；右轮占空比+输出值*/
int16_t Location_Pid_PWM(float set_angle, float current_angle)
{
    static float bias, pwm, integral_bias, last_bias;

    bias = set_angle - current_angle;
    if (bias < -180)
    {
        bias = bias + 360;
    }
    else if (bias > 180)
    {
        bias = bias - 360;
    }

    integral_bias += bias;
    pwm = Position_KP * bias +
          Position_KI * integral_bias +
          Position_KD * (bias - last_bias);
    last_bias = bias;

     if (pwm > 3199) // PWM
     {
         pwm = 3199;
     }
     else if (pwm < (0 - 3199))
     {
         pwm = 0 - 3199;
     }

    return (int16_t)((pwm / 3199.0f) * 100.0f + 0.5f);
}

void goAtoC(){//右转
    if(ACflag==0){
 /*       do{
            uart_deal();
            SpeedL=-Location_Pid_PWM(z1,z);
            SpeedR=Location_Pid_PWM(z1,z);
            Set_Speed(0, SpeedL);
            Set_Speed(1, SpeedR);
        }while(absfloat(absz(z1,z))>3);*/
        do{
            uart_deal();
            if(absfloat(absz(zACtemp,z))>=20){
                SpeedL=30;
                SpeedR=-30;
                Set_Speed(0, SpeedL);
                Set_Speed(1, SpeedR);
            }else if((absfloat(absz(zACtemp,z))>10)&&(absfloat(absz(zACtemp,z))<20)){
                SpeedL=25;
                SpeedR=-25;
                Set_Speed(0, SpeedL);
                Set_Speed(1, SpeedR);
            }else{
                SpeedL=20;
                SpeedR=-20;
                Set_Speed(0, SpeedL);
                Set_Speed(1, SpeedR);
            }
        }while(absfloat(absz(zACtemp,z))>3);
    }
    ACflag=1;
}

void goBtoD(void){//左转
    if(BDflag==0){
         /*do{
            uart_deal();
            SpeedL=-Location_Pid_PWM(z1temp,z);
            SpeedR=Location_Pid_PWM(z1temp,z);
            Set_Speed(0, SpeedL);
            Set_Speed(1, SpeedR);
            oled_show();
         }while(absfloat(absz(z1temp,z))>3);*/
         do{
              uart_deal();
              if(absfloat(absz(zBDtemp,z))>=20){
                 SpeedL=-30;
                 SpeedR=30;
                 Set_Speed(0, SpeedL);
                 Set_Speed(1, SpeedR);
             }else if((absfloat(absz(zBDtemp,z))>10)&&(absfloat(absz(zBDtemp,z))<20)){
                 SpeedL=-25;
                 SpeedR=25;
                 Set_Speed(0, SpeedL);
                 Set_Speed(1, SpeedR);
             }else{
                 SpeedL=-20;
                 SpeedR=20;
                 Set_Speed(0, SpeedL);
                 Set_Speed(1, SpeedR);
             }
         }while(absfloat(absz(zBDtemp,z))>3);
    }
    BDflag=1;
}

float absz(float a,float b){////偏航轴差计算，计算两个角度差值的绝对值
    float c = a - b;
    if (c < -180)
            {
                c = c + 360;
            }
        else if (c > 180)
            {
                c = c - 360;
        }
    return c;
}

void HD_MOTO_T3(void){
    HD_Read();
        if(HD[0]==1 && HD[1]==1 && HD[2]==0 && HD[3]==0 && HD[4]==0 && HD[5]==0 && HD[6]==0 && HD[7]==0)
        {
            SpeedL=-20;
            SpeedR=60;
            Set_Speed(0, SpeedL);
            Set_Speed(1, SpeedR);
            delay_ms(20);
        }else if(HD[0]==1 && HD[1]==0 && HD[2]==0 && HD[3]==0 && HD[4]==0 && HD[5]==0 && HD[6]==0 && HD[7]==0)
        {
            SpeedL=-20;
            SpeedR=60;
            Set_Speed(0, SpeedL);
            Set_Speed(1, SpeedR);
            delay_ms(20);
        }else if(HD[0]==0 && HD[1]==1 && HD[2]==0 && HD[3]==0 && HD[4]==0 && HD[5]==0 && HD[6]==0 && HD[7]==0)
        {
            SpeedL=-20;
            SpeedR=60;
            Set_Speed(0, SpeedL);
            Set_Speed(1, SpeedR);
            delay_ms(20);
        }else if(HD[0]==0 && HD[1]==0 && HD[2]==1 && HD[3]==1 && HD[4]==0 && HD[5]==0 && HD[6]==0 && HD[7]==0)
        {
            SpeedL=-10;
            SpeedR=50;
            Set_Speed(0, SpeedL);
            Set_Speed(1, SpeedR);
            delay_ms(10);
        }else if(HD[0]==0 && HD[1]==0 && HD[2]==1 && HD[3]==0 && HD[4]==0 && HD[5]==0 && HD[6]==0 && HD[7]==0)
        {
            SpeedL=-10;
            SpeedR=50;
            Set_Speed(0, SpeedL);
            Set_Speed(1, SpeedR);
            delay_ms(10);
        }else if(HD[0]==0 && HD[1]==0 && HD[2]==0 && HD[3]==1 && HD[4]==0 && HD[5]==0 && HD[6]==0 && HD[7]==0)
        {
            SpeedL=10;
            SpeedR=30;
            Set_Speed(0, SpeedL);
            Set_Speed(1, SpeedR);
            delay_ms(5);
        }else if(HD[0]==0 && HD[1]==0 && HD[2]==0 && HD[3]==1 && HD[4]==1 && HD[5]==0 && HD[6]==0 && HD[7]==0)
        {
            SpeedL=20;
            SpeedR=20;
            Set_Speed(0, SpeedL);
            Set_Speed(1, SpeedR);
            //delay_ms(1);
        }else if(HD[0]==0 && HD[1]==0 && HD[2]==0 && HD[3]==0 && HD[4]==1 && HD[5]==0 && HD[6]==0 && HD[7]==0)
        {
            SpeedL=30;
            SpeedR=10;
            Set_Speed(0, SpeedL);
            Set_Speed(1, SpeedR);
            delay_ms(5);
        }else if(HD[0]==0 && HD[1]==0 && HD[2]==0 && HD[3]==0 && HD[4]==0 && HD[5]==1 && HD[6]==0 && HD[7]==0)
        {
            SpeedL=50;
            SpeedR=-10;
            Set_Speed(0, SpeedL);
            Set_Speed(1, SpeedR);
            delay_ms(10);
        }else if(HD[0]==0 && HD[1]==0 && HD[2]==0 && HD[3]==0 && HD[4]==1 && HD[5]==1 && HD[6]==0 && HD[7]==0)
        {
            SpeedL=50;
            SpeedR=-10;
            Set_Speed(0, SpeedL);
            Set_Speed(1, SpeedR);
            delay_ms(10);
        }else if(HD[0]==0 && HD[1]==0 && HD[2]==0 && HD[3]==0 && HD[4]==0 && HD[5]==0 && HD[6]==1 && HD[7]==0)
        {
            SpeedL=60;
            SpeedR=-20;
            Set_Speed(0, SpeedL);
            Set_Speed(1, SpeedR);
            delay_ms(20);
        }else if(HD[0]==0 && HD[1]==0 && HD[2]==0 && HD[3]==0 && HD[4]==0 && HD[5]==0 && HD[6]==0 && HD[7]==1)
        {
            SpeedL=60;
            SpeedR=-20;
            Set_Speed(0, SpeedL);
            Set_Speed(1, SpeedR);
            delay_ms(20);
        }else if(HD[0]==0 && HD[1]==0 && HD[2]==0 && HD[3]==0 && HD[4]==0 && HD[5]==0 && HD[6]==1 && HD[7]==1)
        {
            SpeedL=60;
            SpeedR=-20;
            Set_Speed(0, SpeedL);
            Set_Speed(1, SpeedR);
            delay_ms(20);
        }else if(HD[0]==0 && HD[1]==0 && HD[2]==0 && HD[3]==0 && HD[4]==0 && HD[5]==1 && HD[6]==1 && HD[7]==0)
        {
            SpeedL=50;
            SpeedR=-10;
            Set_Speed(0, SpeedL);
            Set_Speed(1, SpeedR);
            delay_ms(10);
        }
        else if(HD[0]==0 && HD[1]==1 && HD[2]==1 && HD[3]==0 && HD[4]==0 && HD[5]==0 && HD[6]==0 && HD[7]==0)
        {
            SpeedL=-10;
            SpeedR=50;
            Set_Speed(0, SpeedL);
            Set_Speed(1, SpeedR);
            delay_ms(10);
        }
        else if(HD[0]==1 && HD[1]==1 && HD[2]==1)
        {
            SpeedL=50;
            SpeedR=-10;
            Set_Speed(0, SpeedL);
            Set_Speed(1, SpeedR);
            delay_ms(10);
        }
        else if(HD[1]==1 && HD[2]==1 && HD[3]==1)
        {
            SpeedL=-10;
            SpeedR=50;
            Set_Speed(0, SpeedL);
            Set_Speed(1, SpeedR);
            delay_ms(10);
        }
        else if(HD[5]==1 && HD[6]==1 && HD[7]==1)
        {
            SpeedL=50;
            SpeedR=-10;
            Set_Speed(0, SpeedL);
            Set_Speed(1, SpeedR);
            delay_ms(10);
        }
        else if(HD[4]==1 && HD[5]==1 && HD[6]==1)
        {
            SpeedL=50;
            SpeedR=-10;
            Set_Speed(0, SpeedL);
            Set_Speed(1, SpeedR);
            delay_ms(10);
        }
        else if(HD[0]==0 && HD[1]==0 && HD[2]==0 && HD[3]==0 && HD[4]==0 && HD[5]==0 && HD[6]==0 && HD[7]==0)
        {
            SpeedL=15;
            SpeedR=15;
            Set_Speed(0, SpeedL);
            Set_Speed(1, SpeedR);
            delay_ms(8);
            HD_Read();
            if(HD[0]==0 && HD[1]==0 && HD[2]==0 && HD[3]==0 && HD[4]==0 && HD[5]==0 && HD[6]==0 && HD[7]==0){
                if((absfloat(absz(z1temp,z))<18)){
                    runflag=0;
                }else if((absfloat(absz(z1,z))<18)){
                    runflag=0;
                }
            }
        }
        else{
            SpeedL=-10;
            SpeedR=10;
            Set_Speed(0, SpeedL);
            Set_Speed(1, SpeedR);
        }
}

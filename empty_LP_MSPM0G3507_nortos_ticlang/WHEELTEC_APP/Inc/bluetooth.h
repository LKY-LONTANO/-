#ifndef __BLUETOOTH_H
#define __BLUETOOTH_H

#include "ti_msp_dl_config.h"

void bt_control(uint8_t recv);
void BT_DAMConfig(void);
void BTBufferHandler(void);

//PID参数发送到APP的标志位
extern uint8_t PID_Send;

#endif




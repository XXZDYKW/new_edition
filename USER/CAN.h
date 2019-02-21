#ifndef __CAN_H_
#define __CAN_H_

#include "main.h"


void My_can_init(void);
void set_yuntai_angle(short yuntai_angle);
void set_capture_angle(short capture_angle);
void CAN1_Configuration(void);
void CAN_Send_Msg2(void);
void CAN_Send_Msg1(short Velocity_M1,short Velocity_M2,short Velocity_M3);
u8 CAN1_Send_Msg(u8* msg,u8 len,u16 ID);
void CAN_Delay_Us(unsigned int t);


#endif


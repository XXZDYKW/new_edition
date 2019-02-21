#ifndef __TIMER_H
#define __TIMER_H
#include "sys.h"

extern u8  TIM5CH1_CAPTURE_STA;		//输入捕获状态		    				
extern u32	TIM5CH1_CAPTURE_VAL;	//输入捕获值  

void TIM5_CH1_Cap_Init(u32 arr,u16 psc);
void Infrared_Init_1(void);

extern u8  TIM2CH2_CAPTURE_STA;		//输入捕获状态		    				
extern u32	TIM2CH2_CAPTURE_VAL;	//输入捕获值  

void TIM2_CH2_Cap_Init(u32 arr,u16 psc);
void Infrared_Init_2(void);

void turn_infrared_on(void);

void TIM3_Int_Init(u16 arr,u16 psc);

extern int flag_bule_tooth;

#endif


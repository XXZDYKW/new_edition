#ifndef __FLYFUN_H__
#define __FLYFUN_H__
#include "sys.h"

void TIM9_PWM_Init(u32 arr,u32 psc);
void FunVelocity_Control(int max_velocity,int min_velocity,int aim_velocity);
void Fun_Init(void);

#endif




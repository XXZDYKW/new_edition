#ifndef __MOTOR_CONTROL_H_
#define __MOTOR_CONTROL_H_

#include "main.h"

typedef struct
{
	float v1;
	float v2;
	float v3;
	//float v4;
}motor_velo;

extern motor_velo link_v;
extern motor_velo turn_v;
extern motor_velo adjust_v;
/*****整体速度*********/
extern motor_velo velocity;

void velocity_calcu(void);
void CMControlLoop(void);
void Stop(void);

extern float L;

#endif


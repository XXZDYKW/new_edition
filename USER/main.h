#ifndef __MAIN_H__
#define __MAIN_H__

#include "stm32f4xx.h"
#include "stm32f4xx_tim.h"
#include "stdlib.h"
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdarg.h>

#include "timer.h"
#include "delay.h"
#include "sys.h"
#include "usart.h"
#include "Flyfun.h"
#include "DT7.h"

typedef struct
{
	float kp;
	float ki;
	float kd;

	float output;
	
	float err;
	float err_last;
	float err_sum;
	float err_dif;
	
	float output_max;
	float i_max;
}PID_Parameter;

#include "Motor_Control.h"
#include "PID.h"
#include "Aim_Calcu.h"
#include "CAN.h"


/***位置坐标和角度结构体***/
typedef struct
{
	float x;
	float y;
	float angle;
}position;

extern position posi;
extern PID_Parameter link_pid;
extern PID_Parameter angle_pid;


#endif


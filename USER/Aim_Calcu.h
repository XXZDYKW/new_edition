#ifndef _AIM_CALCU_H_
#define _AIM_CALCU_H_

#include "main.h"

/**目标参数结构体**/
typedef struct
{
	float x;
	float y;
	float angle;
}Aim_Parame;

extern Aim_Parame aim[30];
extern float broadside_distance;
extern int k;

void aim_set(Aim_Parame *aim,float x,float y,float angle);
void Link_Output(PID_Parameter *pid);
float angle_diff1(void);
float angle_diff2_angle(void);
float angle_diff2_direction(void);
float angle_diff3_direction(void);
void link_velocity(PID_Parameter *pid,float angle);
void turn_velocity(PID_Parameter *pid,float angle);
void adjust_bridge(float P,float velocity_max);
void velocity_calcu2(void);
void adjust_y(void);
void FAST_EST_(void);

#endif


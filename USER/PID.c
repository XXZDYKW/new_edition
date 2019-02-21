#include "main.h" 

/***********PID参数赋值函数***************/
void PID_set(PID_Parameter *PID,float kp,float ki,float kd,float output_max,float i_max)
{
	PID->kp = kp;
	PID->ki = ki;
	PID->kd = kd;
	PID->output_max = output_max;
	PID->i_max = i_max;
}

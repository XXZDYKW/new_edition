#ifndef _PID_H_
#define _PID_H_

#include "main.h"

void PID_set(PID_Parameter *pid,float kp,float ki,float kd,float output_max,float i_max);

#endif


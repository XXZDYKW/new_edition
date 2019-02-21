#include "main.h"
extern PID_Parameter link_pid;
motor_velo link_v;
motor_velo turn_v;
motor_velo adjust_v;
/*****整体速度*********/
motor_velo velocity;
float L = 600;

void velocity_calcu( void )
{
	velocity.v1 = -1*link_v.v1 - turn_v.v1 ;
	velocity.v2 = -1*link_v.v2 - turn_v.v2 ;
	velocity.v3 = -1*link_v.v3 - turn_v.v3 ;
	float vm = 0;
	float prot_K;
	if(abs(velocity.v1)>vm)
		vm=abs(velocity.v1);
	if(abs(velocity.v2)>vm)
		vm=abs(velocity.v2);
	if(abs(velocity.v3)>vm)
		vm=abs(velocity.v3);
	if(vm>1280)
	{
		prot_K = 1280/vm;
		velocity.v1 = velocity.v1*prot_K;
		velocity.v2 = velocity.v2*prot_K;
	  velocity.v3 = velocity.v3*prot_K;
	}
}

void velocity_calcu2(void)/***拐弯转向欧米噶***/
{
	velocity.v1 += link_pid.output*L/700;
	velocity.v2 += link_pid.output*L/700;
	velocity.v3 += link_pid.output*L/700;
	float vm = 0;
	float prot_K;
	if(abs(velocity.v1)>vm)
		vm=abs(velocity.v1);
	if(abs(velocity.v2)>vm)
		vm=abs(velocity.v2);
	if(abs(velocity.v3)>vm)
		vm=abs(velocity.v3);
	if(vm>1280)
	{
		prot_K = 1280/vm;
		velocity.v1 = velocity.v1*prot_K;
		velocity.v2 = velocity.v2*prot_K;
	  velocity.v3 = velocity.v3*prot_K;
	}
}

void CMControlLoop(void)
{ 
  CAN_Send_Msg1((short)velocity.v1,(short)velocity.v2,(short)velocity.v3);
}

void Stop(void)
{ 
	CAN_Send_Msg1(0,0,0);
}


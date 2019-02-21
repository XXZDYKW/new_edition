#include "main.h"


extern int k_max;

void aim_set(Aim_Parame *aim,float x,float y,float angle)
{
	aim->x = x;
	aim->y = y;
	aim->angle = angle;
}

/****目标结构体设置****/
Aim_Parame aim[30];
int k;


/*************************************根据距离算出Link_output************************************************/
extern int flag_init;
extern int flag_round_two;

void Link_Output(PID_Parameter *pid)
{
	if(flag_init||flag_round_two)//初始化时或交接后
	{
		pid->err_last = pid->err ;
		pid->err = (float)sqrt((double)((aim[k].y - posi.y)*(aim[k].y - posi.y)+(aim[k].x - posi.x)*(aim[k].x - posi.x)));
		pid->err_sum  += pid->err;
		pid->err_dif = pid->err - pid->err_last;
		if(pid->err_sum>=pid->i_max)pid->err_sum=pid->i_max;
		if(pid->err_sum<=-1*pid->i_max)pid->err_sum=-1*pid->i_max;
		pid->output = pid->kp*pid->err + pid->ki*pid->err_sum +pid->kd*pid->err_dif;
	}
	else//否则交接之前
	{
		if(k <= 7)
			pid->output = pid->output_max;
		else
		{
			pid->err_last = pid->err ;
			pid->err = (float)sqrt((double)((aim[k].y + 2490 - posi.y)*(aim[k].y + 2490 - posi.y)+(aim[k].x - posi.x)*(aim[k].x - posi.x)));//停车时的坐标判断，因为此程序是手动交接，2850的值其实是在交接之前就停车了
			pid->err_sum  += pid->err;
			pid->err_dif = pid->err - pid->err_last;
			if(pid->err_sum>=pid->i_max)pid->err_sum=pid->i_max;
			if(pid->err_sum<=-1*pid->i_max)pid->err_sum=-1*pid->i_max;
			pid->output = pid->kp*pid->err + pid->ki*pid->err_sum +pid->kd*pid->err_dif;
		}
  }
	
	
	if(pid->output>=pid->output_max)pid->output=pid->output_max;
	if(pid->output<=-1*pid->output_max)pid->output=-1*pid->output_max;
}

/***************计算目前坐标和目标坐标的角度，返回差值*******************************************************/

float angle_diff1(void)
{
	float angle;
	float angle_coord;
	if((aim[k].x - posi.x)>0)
	{
		angle_coord = (float)(57.2958)*(float)atan((double)((aim[k].y - posi.y) / (aim[k].x - posi.x)));
	}
	else if((aim[k].x - posi.x)<0)
	{
		angle = (float)(57.2958)*(float)atan((double)((aim[k].y - posi.y) / (aim[k].x - posi.x)));
		if(angle>0)
			angle_coord = angle-180;
		else
			angle_coord = angle+180;
	}
	else if((aim[k].x - posi.x) == 0)
	{
		if((aim[k].y - posi.y)>0)
			angle_coord = 90;
		else if((aim[k].y - posi.y)<0)
			angle_coord = -90;
		else angle_coord = 0;
	}
	return(angle_coord - posi.angle);
}

/*******************************计算k=2时的前进的方向******************************************************/
float angle_diff2_angle(void)
{
	float angle_coord;
	angle_coord = (float)(57.2958)*(float)atan((double)-0.6*cos((posi.x-580)*3.14/1500));
	if(posi.x>4330)          //posi.x大于4330之后车头向前，4330与580相对应
		angle_coord = 0;
	return(angle_coord);
}


float angle_diff2_direction(void)
{
	float angle_coord;
	angle_coord = (float)(57.2958)*(float)atan((double)(-350*sin((posi.x-750+200)*3.14/1500) - posi.y - 725) / 200);
	return(angle_coord - posi.angle);
}

/*******************************当k=3时，准备进入桥段******************************************************/
void adjust_bridge(float P,float velocity_max)
{
	float adjust_err;
	adjust_err = P * (posi.y - aim[k].y);
	if(adjust_err>velocity_max)
		adjust_err = velocity_max;
	if(adjust_err<-1*velocity_max)
		adjust_err = -1*velocity_max;
	if(abs(adjust_err) <= 10)
		adjust_err = 0;
	velocity.v1 -= adjust_err/2;
	velocity.v3 -= adjust_err/2;
	velocity.v2 += adjust_err;
}

/**********************利用上面的计算Link_output的函数以及计算前进角度函数，得出三个点击的速度分配情况******/
void link_velocity(PID_Parameter *pid,float angle)
{
	angle /= 57.2958;
	link_v.v1 = -1*pid->output*(float)(cos((double)angle)*1.732/2 + sin((double)angle)/2);
	link_v.v2 = pid->output*sin((double)angle);
	link_v.v3 = -1*pid->output*(float)(-1*cos((double)angle)*1.732/2 + sin((double)angle)/2);
}


/***************电机最大速度是1280，此函数将解算后速度最大的电机速度提升到指定值****************/
void FAST_EST_()
{
	float vm = 0;
	float fast_K;
	if(abs(velocity.v1)>vm)
		vm=abs(velocity.v1);
	if(abs(velocity.v2)>vm)
		vm=abs(velocity.v2);
	if(abs(velocity.v3)>vm)
		vm=abs(velocity.v3);
		if(flag_round_two)//交接后
	{
		if(vm>700)//速度待调
		{
			fast_K = 700/vm;
			velocity.v1 = velocity.v1*fast_K;
			velocity.v2 = velocity.v2*fast_K;
			velocity.v3 = velocity.v3*fast_K;
		}
	}
	else//交接前
	{
		if(k == 8)    //减速准备交接
		{
			if(vm>700)
			{
				fast_K = 700/vm;
				velocity.v1 = velocity.v1*fast_K;
				velocity.v2 = velocity.v2*fast_K;
				velocity.v3 = velocity.v3*fast_K;
			}
		}
		else
		{
				fast_K = 700/vm;
				velocity.v1 = velocity.v1*fast_K;
				velocity.v2 = velocity.v2*fast_K;
				velocity.v3 = velocity.v3*fast_K;
		}
	}
}



/*********************************旋转角度的v的计算*********************************************************/
void turn_velocity(PID_Parameter *pid,float angle)
{
	pid->err_last = pid->err;
	pid->err = angle - posi.angle;
	if(pid->err>180)pid->err=pid->err-360;
	else
	if(pid->err<-180)pid->err=pid->err+360;
	else
	pid->err_dif = pid->err - pid->err_last;
//计算PID的error和error的差值
	
	pid->err_sum  += pid->err;
	if(pid->err_sum>=pid->i_max)pid->err_sum=pid->i_max;
	if(pid->err_sum<=-1*pid->i_max)pid->err_sum=-1*pid->i_max;
//计算PID的error总和，消除静差
	
	pid->output = pid->kp*pid->err + pid->ki*pid->err_sum +pid->kd*pid->err_dif;
	
	
	if(pid->output>=pid->output_max)pid->output=pid->output_max;
	if(pid->output<=-1*pid->output_max)pid->output=-1*pid->output_max;
	
	
	turn_v.v1 = turn_v.v2 = turn_v.v3 = pid->output;
}


/*****************************抓取拐骨时调整轨迹****************************/
void adjust_y()
{
	float adjust_err;
	adjust_err = 2*(aim[k].y - posi.y);
	if(adjust_err>200)
		adjust_err = 200;
	if(adjust_err<-200)
		adjust_err = -200;
	velocity.v1 -= adjust_err/2;
	velocity.v3 -= adjust_err/2;
	velocity.v2 += adjust_err;
}



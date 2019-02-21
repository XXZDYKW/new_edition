/************Dream chaser************/
/***************Xxz******************/

#include "main.h"

PID_Parameter link_pid;       //前进速度的PID
PID_Parameter angle_pid;      //车头转向的PID

extern long long Infrared_distance_1;//后面的红外数据，靠墙稳定值750以下
extern long long Infrared_distance_2;//前面的红外数据，靠墙稳定值750以下

int k_max;               //底盘跑点，k_max是最后一个点的序号加一

/****使用DT7遥控器*****/
float ahead_velo;  //前进遥控参数
float aside_velo;  //左右摇控参数
float turn_velo;   //旋转参数
extern DBUS dbus;  //遥控参数

u8 canbuf[2];      //电磁阀CAN发送数据数组

int flag_round_two = 0;//交接完成标志
int flag_init;

short m3508_capture_angle = 0;  //抓取拐骨位置环电机角度，正常在车上时角度为零，打开时角度约为160度
short m2006_yuntai_angle = 0;   //云台角度，朝正前方时角度为零，向左转为负角度，向右正


int main(void)
{
/********************************************初始化*********************************************************/
	delay_init(168);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);    //中断优先级初始化配置
	My_can_init();           //CAN初始化
	USART2_Init(115200);     //串口而用于接收全场定位模块的数据
	
/****************风机初始化****************/
	int max_velocity=1999;   //风机最大值
	int min_velocity=999;    //最小值
	Fun_Init();              //初始化PWM
	DT7_init();              //遥控器初始化
	
/***********两个位置环电机初始化**********/
	set_yuntai_angle(0);     //设置云台角度等于零
	set_capture_angle(0);    //设置抓取角度

/***************红外初始化****************/
/*红外测距20MS返回一次数据，在中断中对数据进行处理，初始化后直接使用全局变量的值即可，占用控制器资源少*/
	Infrared_Init_1();			 //红外初始化
	Infrared_Init_2();       //红外初始化	
	//turn_infrared_on();    //开启测距，此程序中没有用到测距，所以没有打开测距
/*关闭风机，等待开始比赛*/
	FunVelocity_Control(max_velocity, min_velocity,1000);//关闭风机
	
/******************************气缸初始化********************************************************************/
//旋转气缸转到零度#########################################################################################
//交接伸缩气缸缩回#########################################################################################
	canbuf[0]=0x03;
	canbuf[1]=0x01;
	CAN1_Send_Msg(canbuf,2,0x301);
	delay_ms(1);
//交接夹取气缸夹紧#########################################################################################
	canbuf[0]=0x08;
	canbuf[1]=0x00;
	CAN1_Send_Msg(canbuf,2,0x301);
	delay_ms(1);
	delay_ms(100);
	canbuf[1]=0x01;
	CAN1_Send_Msg(canbuf,2,0x301);
	delay_ms(1);
//升降气缸升起#############################################################################################
	canbuf[0]=0x06;
	canbuf[1]=0x01;
	CAN1_Send_Msg(canbuf,2,0x301);
	delay_ms(1);
//发射夹取气缸打开#############################################################################################
	canbuf[0]=0x02;
	canbuf[1]=0x01;
	CAN1_Send_Msg(canbuf,2,0x301);
	delay_ms(1);
	canbuf[0]=0x04;
	canbuf[1]=0x00;
	CAN1_Send_Msg(canbuf,2,0x301);
	delay_ms(1);
//发射小气缸伸出###########################################################################################
	canbuf[0]=0x05;
	canbuf[1]=0x01;
	CAN1_Send_Msg(canbuf,2,0x301);
	delay_ms(1);
//发射气缸缩回#############################################################################################
	canbuf[0]=0x01;
	canbuf[1]=0x01;
	CAN1_Send_Msg(canbuf,2,0x301);
	delay_ms(1);
	
/******手动放入令牌********/	
//交接夹取气缸夹紧#########################################################################################
	canbuf[0]=0x08;
	canbuf[1]=0x00;
	CAN1_Send_Msg(canbuf,2,0x301);
	delay_ms(100);
	canbuf[1]=0x01;
	CAN1_Send_Msg(canbuf,2,0x301);
	delay_ms(1);
//交接夹取气缸打开##########################################################################################
	canbuf[0]=0x07;
	canbuf[1]=0x00;
	CAN1_Send_Msg(canbuf,2,0x301);
	delay_ms(100);
	canbuf[1]=0x01;
	CAN1_Send_Msg(canbuf,2,0x301);
	delay_ms(1);
	delay_ms(5000);//五秒钟时间放入令牌
//交接夹取气缸夹紧#########################################################################################
	canbuf[0]=0x08;
	canbuf[1]=0x00;
	CAN1_Send_Msg(canbuf,2,0x301);
	delay_ms(100);
	canbuf[1]=0x01;
	CAN1_Send_Msg(canbuf,2,0x301);
	delay_ms(5000);         //等待全场定位模块初始化完成  
	set_yuntai_angle(-20);   //初始化完成后云台旋转
/*******************************检测遥控器可用**************************************************************/
	float vm = 0;       //限速用，见函数FAST_EST_();
	float fast_K;       //限速用，见函数FAST_EST_();
	dbus.rc.ch0 = dbus.rc.ch1 = dbus.rc.ch2 = dbus.rc.ch3 = 1024;//给模拟量赋中间值，防止遥控器没有返回数据失控
	while(1)
	{
		ahead_velo = (dbus.rc.ch3 - 1024)*30/660;
		aside_velo = (dbus.rc.ch2 - 1024)*30/660;
		turn_velo = -1*(dbus.rc.ch0 - 1024)*20/660;
		link_v.v1 = -1*ahead_velo*1.732/2 + aside_velo/2;
		link_v.v2 = -1*aside_velo;
		link_v.v3 = ahead_velo*1.732/2 + aside_velo/2;
		turn_v.v1 = turn_velo;
		turn_v.v2 = turn_velo;
		turn_v.v3 = turn_velo;
		velocity_calcu();
		if(abs(velocity.v1)>vm)
			vm=abs(velocity.v1);
		if(abs(velocity.v2)>vm)
			vm=abs(velocity.v2);
		if(abs(velocity.v3)>vm)
			vm=abs(velocity.v3);
		if(vm>700)
		{
			fast_K = 700/vm;
			velocity.v1 = velocity.v1*fast_K;
			velocity.v2 = velocity.v2*fast_K;
			velocity.v3 = velocity.v3*fast_K;
		}
		CMControlLoop();
		if(dbus.rc.s1==1)
			break;
	}
	
/******************************************先转负三十度等待开始*********************************************/
  k = 1;
	PID_set(&link_pid,0.5,0,0,100,1);
	PID_set(&angle_pid,10,0,0,70,1);
	aim_set(&aim[1],0,-100,-32);
	
	while(dbus.rc.s1!=1);//将左侧开关向上拨，底盘转31°,并向右前进0.1m
	flag_init = 1;
	
	while(1)
	{
		if(flag1)
		{
			Link_Output(&link_pid);
			link_velocity(&link_pid,angle_diff1()); 
			turn_velocity(&angle_pid,aim[k].angle); 
			velocity_calcu();
			CMControlLoop();
		}
		
		if(abs(aim[k].x-posi.x)<10&&abs(aim[k].y-posi.y)<10&&abs(aim[k].angle-posi.angle)<2)
			break;
	}
	
	flag_init = 0;
	Stop();
	
/************************************准备开始**********************************************/
	k = 1;                  //k从一计数
	k_max = 9;              //底盘跑点，k_max是最后一个点的序号加一
	int flag2 = 0;          //到达某个坐标标志

/*****************************************设置PID参数*******************************************************/
  PID_set(&link_pid,1.5,0,0,1000,1);      //前进的PID
  PID_set(&angle_pid,15,0,0,400,1);     //角度的PID
/****************************************确定行进的路线*****************************************************/
  
	aim_set(&aim[1],700,-620,-31);        //跑直线，切入正弦
	aim_set(&aim[2],4900,-725,0);         //跑正弦过森林
	aim_set(&aim[3],5900,-780,0);         //准备进桥阶段
	aim_set(&aim[4],7250,-725,0);         //过桥，7250可能需要调整
	aim_set(&aim[5],7900,-1500,-90);      //转弯
  aim_set(&aim[6],7850,-2800,-117);	    //准备靠边
	aim_set(&aim[7],7750,-4400,-118);	    //准备靠边
	aim_set(&aim[8],7100,-7500,-118);	    //靠墙阶段，此程序纯物理靠墙
/*物理靠墙解释：让车向右前方行驶，同时角度略大于-120度，
  此程序让车朝向(7100,-7000)的方向前进，而实际停车处Y坐标在-5000左右，
	具体停车调整代码见函数void Link_Output(PID_Parameter *pid)*/
	
/**********************************************等待遥控，开启风机*******************************************/
  while(dbus.rc.s1!=3);//将左侧开关拨到中间位置，开始比赛
	dbus.rc.ch0 = dbus.rc.ch1 = dbus.rc.ch2 = dbus.rc.ch3 = 1024;//给模拟量赋中间值，防止遥控器没有返回数据失控
	FunVelocity_Control(max_velocity, min_velocity,1400);             //开风机
	delay_ms(3000);//延时是安全提醒，比赛时删除
	set_yuntai_angle(0);
	
/****************************************计算电机的转速*****************************************************/	
	while(1)
	{
		if(flag1)
		{
      if(k==5)/******转弯部分********/
			{
				turn_v.v1 = turn_v.v2 = turn_v.v3 = 0;
				link_velocity(&link_pid,0); 
				velocity_calcu(); 
				velocity_calcu2();    //在匀速前进的速度中加入旋转角速度
				FAST_EST_();          //速度调整，电机最大速度为1280
				CMControlLoop();      //使用CAN总线发送电机速度控制信号
				flag1 = 0;
			}
			else
			{
				Link_Output(&link_pid);      //计算距离PID的OUTPUT，主要用于停车时减速，不停车时匀速前进，实际的OUTPUT只是提供一个与转弯速度比例关系，具体数值的意义不大
				if(k==2)                     //跑正弦，过森林
				{
					link_velocity(&link_pid,angle_diff2_direction());   //跑正弦前进方向的速度分量
					turn_velocity(&angle_pid,angle_diff2_angle());      //跑正弦车头方向即转弯速度分量
				}
/*跑正弦时旋转速度分量的正弦函数滞后于前进方向速度函数大约20cm相位差，目的是保证转向的及时性，此经验是测试得出，有兴趣的同学可以去掉相位差尝试一下*/
				else
				{
					link_velocity(&link_pid,angle_diff1());    //其他情况跑直线，前进方向的速度分量
				  turn_velocity(&angle_pid,aim[k].angle);    //车头转向速度
				}
				velocity_calcu();        //以上三个速度加和
				if(k==3)
					adjust_bridge(1.0,400);
				if(k==4)
					adjust_bridge(0.5,300);
				FAST_EST_();             //将速度按比例进行调整
				CMControlLoop();         //驱动电机 
				flag1 = 0;
			}
		}
		
/*****************************************判断是否到达位置*****************************************************/
		if(k==5)//判断转弯是否结束
		{
			if((aim[k].y-posi.y)>-200)  //达到坐标判断
				flag2 = 1;
			if(posi.angle<-60)       //到达角度判断
				flag2 = 1;
		}
		else if(k<5)//转弯之前的判断
		{
			if((aim[k].x-posi.x)<50)  //拐弯之前的到达目标判断用x的数值
			flag2 = 1;
		}
		else if(k==8)         //停车点用y坐标判断
		{
			if(posi.y<-5000)    //-5200有待调整，此数值与void Link_Output(PID_Parameter *pid)中的2290相对应
			  flag2 = 1;
		}
		else
		{
		  if((aim[k].y-posi.y)>-50)   //拐弯之后的到达目标判断用y的数值
			flag2 = 1;				
		}	
/*********************************************到达坐标后的调整******************************************************/		
		if(flag2)
		{
			flag2 = 0;
			k++;
			
			if(k==2)
			  PID_set(&angle_pid,35,0,0,200,1);     //角度的PID，跑正弦时给一个较大的P、OUTPUT_MAX值
			if(k==3)
			{
				PID_set(&angle_pid,20,0,0,300,1);     //角度的PID，跑圆弧时的P、OUTPUT_MAX值不需太大
				FunVelocity_Control(max_velocity, min_velocity,1500);//加大风机进入桥段
			}
			if(k==4)
				PID_set(&angle_pid,15,0,0,200,1);     //角度的PID，进桥后
			if(k==6)
				PID_set(&angle_pid,10,0,0,170,1);     //角度的PID，转弯后，给一个比较小的P、OUTPUT_MAX值防止转过撞墙
			if(k==7)
			{
				PID_set(&angle_pid,10,0,0,200,1);     //角度的PID，准备交接
				FunVelocity_Control(max_velocity, min_velocity,1000);//关风机
			}
			if(k==8)
			{
				PID_set(&angle_pid,35,0,0,350,1);     //角度的PID，靠墙的PID，给定较大的P值，使角度能够稳定保持，即稳定靠墙
/****************伸出令牌，降下令牌*****************************************************************************/
//交接伸缩气缸伸出#############################################################################################
//旋转气缸转到三十度###########################################################################################
				canbuf[0]=0x03;
				canbuf[1]=0x00;
				CAN1_Send_Msg(canbuf,2,0x301);
				delay_ms(1);
//升降气缸下降#################################################################################################
				canbuf[0]=0x06;
				canbuf[1]=0x00;
				CAN1_Send_Msg(canbuf,2,0x301);
				delay_ms(1);
			}
			if(k>=k_max)
			{
				FunVelocity_Control(max_velocity, min_velocity,1000);//关风机
				break;
			}
		}
	}
	
	FunVelocity_Control(max_velocity, min_velocity,1000);//关风机
	Stop();
	
	
	delay_ms(3000);//看停车效果，比赛删除 
	
//旋转气缸转到零度#########################################################################################
//交接伸缩气缸缩回#########################################################################################
	canbuf[0]=0x03;
	canbuf[1]=0x01;
	CAN1_Send_Msg(canbuf,2,0x301);
	delay_ms(1);
	
	
/*********************************************抓取发射拐骨*********************************************************/
	Stop();
	
	k=1;
	k_max=9;
	int count_flag1 = 0;//全场地位模块每5MS返回一次数据，通过计数全场定位模块返回数据的次数来进行粗略计时，减小CPU计时压力
	int flag_k7 = 1; //k=7y坐标调整完成标志
	flag_round_two = 1;//第二回合
	
	PID_set(&link_pid,2,0,0,300,0);//等待二车到达高山驿的期间速度不用很快
	PID_set(&angle_pid,15,0,0,100,1);
	
	aim_set(&aim[1],8500,-4700,90);//准备
	aim_set(&aim[2],8500,-4600,90);//第一次夹取
	aim_set(&aim[3],7800,-3400,180);//进入等待区
	aim_set(&aim[4],4000,-3200,180);//进入发射区
	aim_set(&aim[5],8200,-3200,180);//第二次夹取
  aim_set(&aim[6],4000,-3200,180);//第二次进入发射区
	aim_set(&aim[7],8200,-2800,180);//第三次夹取
	aim_set(&aim[8],4000,-3200,180);//第三次进入发射区
	
	
	while(1)
	{
    if(flag1)
		{
			flag1=0;
			Link_Output(&link_pid);
			link_velocity(&link_pid,angle_diff1());//跑直线
			turn_velocity(&angle_pid,aim[k].angle);
			velocity_calcu();
			if(k==7&&flag_k7)
			  adjust_y(); //第三次抓取时y坐标调整
			FAST_EST_();
			CMControlLoop();
			
			if(k==4) //防止忘记抬升夹取
			{
				count_flag1++;
				if(count_flag1==100)
					set_capture_angle(0);
			}
			
			if(k==5||k==7) //向抓取点前进的过程中放下抓取装置
			{
				count_flag1++;
				if(count_flag1==300)
					set_capture_angle(140);
			}
			
			if(k==6||k==8)   //k=6的时候一边前进，一边进行气缸操作节省时间
			{
				count_flag1++;
				if(count_flag1==60)//0.3S时夹取装置抬升
					set_capture_angle(0);
				if(count_flag1==300)//1500mS时转动云台
				{
					set_yuntai_angle(-45);
//发射小气缸缩回###########################################################################################
					canbuf[0]=0x05;
					canbuf[1]=0x00;
					CAN1_Send_Msg(canbuf,2,0x301);
					delay_ms(1);
				}
				if(count_flag1==400)//2S打开发射夹取的气缸
				{
//发射夹取气缸打开#############################################################################################
					canbuf[0]=0x02;
					canbuf[1]=0x01;
					CAN1_Send_Msg(canbuf,2,0x301);
					delay_ms(1);
					canbuf[0]=0x04;
					canbuf[1]=0x00;
					CAN1_Send_Msg(canbuf,2,0x301);
					delay_ms(1);
				}
			}
		}

/*************************************到达目标点判断*******************************************************/
		if(abs(aim[k].y-posi.y)<15&&k==7)
			flag_k7 = 0;
		
/*小车在取拐骨的时候可能会出现被拐骨卡住，无法到达目标点的情况，这时候将遥控器两个按键掰向上，同时向上推右侧摇杆，即可进入遥控状态*/
		if((k==2||k==5||k==7)&&dbus.rc.s1==1&&dbus.rc.s2==1&&dbus.rc.ch1>1600)
			flag2 = 1;
		
		if(abs(aim[k].x-posi.x)<15&&abs(aim[k].y-posi.y)<15&&abs(aim[k].angle-posi.angle)<2)
			flag2=1;

/**************************************到达目标点后的调整**************************************************/
		if(flag2)
		{
			flag2=0;
			k++;
			
			if(k==2)//第一次夹取
				set_capture_angle(140);//放下夹取装置，但没有完全放下
			
			if(k==3||k==6||k==8)
			{
				Stop();
				count_flag1 = 0;//计数清零
				set_capture_angle(160);//放下夹取装置
//发射夹取气缸打开#############################################################################################
				canbuf[0]=0x02;
				canbuf[1]=0x01;
				CAN1_Send_Msg(canbuf,2,0x301);
				delay_ms(1);
				canbuf[0]=0x04;
				canbuf[1]=0x00;
				CAN1_Send_Msg(canbuf,2,0x301);
				delay_ms(1000);
/**************************************手动夹取*********************************************************
（1）一车每次到达夹取点时，夹取气缸夹紧后小车自动进入遥控模式；
（2）S1向上拨，抓取装置放下，向下拨抓取装置升起；
（3）S2向上推夹取拐骨气缸打开，向下推夹取拐骨气缸夹紧；
（3）左摇杆控制前后、左右前进，前进方向可以叠加，右摇杆控制转向；
（4）S1、S2都处于中间位置时，将右边的摇杆向上推，小车进入自动模式。*/
/**￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥**/
        while(1)
				{
					ahead_velo = (dbus.rc.ch3 - 1024)*300/660;
					aside_velo = (dbus.rc.ch2 - 1024)*300/660;
					turn_velo = -1*(dbus.rc.ch0 - 1024)*200/660;
					link_v.v1 = -1*ahead_velo*1.732/2 + aside_velo/2;
					link_v.v2 = -1*aside_velo;
					link_v.v3 = ahead_velo*1.732/2 + aside_velo/2;
					turn_v.v1 = turn_velo;
					turn_v.v2 = turn_velo;
					turn_v.v3 = turn_velo;
					velocity_calcu();
					if(abs(velocity.v1)>vm)
						vm=abs(velocity.v1);
					if(abs(velocity.v2)>vm)
						vm=abs(velocity.v2);
					if(abs(velocity.v3)>vm)
						vm=abs(velocity.v3);
					if(vm>700)
					{
						fast_K = 700/vm;
						velocity.v1 = velocity.v1*fast_K;
						velocity.v2 = velocity.v2*fast_K;
						velocity.v3 = velocity.v3*fast_K;
					}
					CMControlLoop();
					
					if(dbus.rc.s1==1&&m3508_capture_angle!=160)//左边开关向上掰，抓取装置放下
						set_capture_angle(160);
					if(dbus.rc.s1==2&&m3508_capture_angle!=0)//左边开关向下掰，抓取装置升起
						set_capture_angle(0);
					
					if(dbus.rc.s2==1&&canbuf[0]!=0x04)//右边开关向上掰，夹取气缸打开
					{
//发射夹取气缸打开#############################################################################################
						canbuf[0]=0x02;
						canbuf[1]=0x01;
						CAN1_Send_Msg(canbuf,2,0x301);
						delay_ms(1);
						canbuf[0]=0x04;
						canbuf[1]=0x00;
						CAN1_Send_Msg(canbuf,2,0x301);
						delay_ms(1);
					}
					if(dbus.rc.s2==2&&canbuf[0]!=0x02)//右边开关向下掰，夹取气缸吸合
					{
//发射夹取气缸夹紧#############################################################################################
						canbuf[0]=0x04;
						canbuf[1]=0x01;
						CAN1_Send_Msg(canbuf,2,0x301);
						delay_ms(1);
						canbuf[0]=0x02;
						canbuf[1]=0x00;
						CAN1_Send_Msg(canbuf,2,0x301);
						delay_ms(1);
					}
					
					if(dbus.rc.s1==3&&dbus.rc.s2==3&&dbus.rc.ch1>1600)//两个开关都处于中间位置，然后向前推右侧摇杆，小车进入自动模式
						break;
				}
/**￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥**/
			
//发射夹取气缸夹紧#############################################################################################
				canbuf[0]=0x04;
				canbuf[1]=0x01;
				CAN1_Send_Msg(canbuf,2,0x301);
				delay_ms(1);
				canbuf[0]=0x02;
				canbuf[1]=0x00;
				CAN1_Send_Msg(canbuf,2,0x301);
				delay_ms(1);
			}
			
			if(k==4)//进入发射区
			{
				Stop();
				count_flag1 = 0;//计数清零
				set_yuntai_angle(-45);//云台角度
				PID_set(&link_pid,1,0,0,500,0);   //进入发射区的速度
	      PID_set(&angle_pid,15,0,0,200,1);   //进入发射区速度
//发射小气缸缩回###############################################################################################
				canbuf[0]=0x05;
				canbuf[1]=0x00;
				CAN1_Send_Msg(canbuf,2,0x301);
				delay_ms(1000);
//发射夹取气缸打开#############################################################################################
				canbuf[0]=0x02;
				canbuf[1]=0x01;
				CAN1_Send_Msg(canbuf,2,0x301);
				delay_ms(1);
				canbuf[0]=0x04;
				canbuf[1]=0x00;
				CAN1_Send_Msg(canbuf,2,0x301);
				while(dbus.rc.s1==2||dbus.rc.s1==3);//左边开关向上掰，进入发射区
			}
			if(k==5||k==7||k==9)//发射
			{
				Stop();
        count_flag1 = 0;//计数清零
//发射夹取气缸打开#############################################################################################
				canbuf[0]=0x02;
				canbuf[1]=0x01;
				CAN1_Send_Msg(canbuf,2,0x301);
				delay_ms(1);
				canbuf[0]=0x04;
				canbuf[1]=0x00;
				CAN1_Send_Msg(canbuf,2,0x301);
				delay_ms(200);//以上夹取气缸打开在实际测试后可删除
//发射气缸推出#################################################################################################
				canbuf[0]=0x01;
				canbuf[1]=0x00;
				CAN1_Send_Msg(canbuf,2,0x301);
				delay_ms(2000);//可以缩短时间
//发射气缸缩回#################################################################################################
				canbuf[0]=0x01;
				canbuf[1]=0x01;
				CAN1_Send_Msg(canbuf,2,0x301);
				delay_ms(1);
//发射小气缸伸出###############################################################################################
				canbuf[0]=0x05;
				canbuf[1]=0x01;
	    	CAN1_Send_Msg(canbuf,2,0x301);
				delay_ms(1);
				set_yuntai_angle(0);
			}
			
			if(k==k_max)
				break;
		}
	}
	
	FunVelocity_Control(max_velocity, min_velocity,1000);//关风机
	Stop();//保持不动
	
	while(1)
	{
		ahead_velo = (dbus.rc.ch3 - 1024)*300/660;
		aside_velo = (dbus.rc.ch2 - 1024)*300/660;
		turn_velo = -1*(dbus.rc.ch0 - 1024)*200/660;
		link_v.v1 = -1*ahead_velo*1.732/2 + aside_velo/2;
		link_v.v2 = -1*aside_velo;
		link_v.v3 = ahead_velo*1.732/2 + aside_velo/2;
		turn_v.v1 = turn_velo;
		turn_v.v2 = turn_velo;
		turn_v.v3 = turn_velo;
		velocity_calcu();
		if(abs(velocity.v1)>vm)
			vm=abs(velocity.v1);
		if(abs(velocity.v2)>vm)
			vm=abs(velocity.v2);
		if(abs(velocity.v3)>vm)
			vm=abs(velocity.v3);
		if(vm>700)
		{
			fast_K = 700/vm;
			velocity.v1 = velocity.v1*fast_K;
			velocity.v2 = velocity.v2*fast_K;
			velocity.v3 = velocity.v3*fast_K;
		}
		CMControlLoop();
	}
}


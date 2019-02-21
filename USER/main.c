/************Dream chaser************/
/***************Xxz******************/

#include "main.h"

PID_Parameter link_pid;       //ǰ���ٶȵ�PID
PID_Parameter angle_pid;      //��ͷת���PID

extern long long Infrared_distance_1;//����ĺ������ݣ���ǽ�ȶ�ֵ750����
extern long long Infrared_distance_2;//ǰ��ĺ������ݣ���ǽ�ȶ�ֵ750����

int k_max;               //�����ܵ㣬k_max�����һ�������ż�һ

/****ʹ��DT7ң����*****/
float ahead_velo;  //ǰ��ң�ز���
float aside_velo;  //����ҡ�ز���
float turn_velo;   //��ת����
extern DBUS dbus;  //ң�ز���

u8 canbuf[2];      //��ŷ�CAN������������

int flag_round_two = 0;//������ɱ�־
int flag_init;

short m3508_capture_angle = 0;  //ץȡ�չ�λ�û�����Ƕȣ������ڳ���ʱ�Ƕ�Ϊ�㣬��ʱ�Ƕ�ԼΪ160��
short m2006_yuntai_angle = 0;   //��̨�Ƕȣ�����ǰ��ʱ�Ƕ�Ϊ�㣬����תΪ���Ƕȣ�������


int main(void)
{
/********************************************��ʼ��*********************************************************/
	delay_init(168);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);    //�ж����ȼ���ʼ������
	My_can_init();           //CAN��ʼ��
	USART2_Init(115200);     //���ڶ����ڽ���ȫ����λģ�������
	
/****************�����ʼ��****************/
	int max_velocity=1999;   //������ֵ
	int min_velocity=999;    //��Сֵ
	Fun_Init();              //��ʼ��PWM
	DT7_init();              //ң������ʼ��
	
/***********����λ�û������ʼ��**********/
	set_yuntai_angle(0);     //������̨�Ƕȵ�����
	set_capture_angle(0);    //����ץȡ�Ƕ�

/***************�����ʼ��****************/
/*������20MS����һ�����ݣ����ж��ж����ݽ��д�����ʼ����ֱ��ʹ��ȫ�ֱ�����ֵ���ɣ�ռ�ÿ�������Դ��*/
	Infrared_Init_1();			 //�����ʼ��
	Infrared_Init_2();       //�����ʼ��	
	//turn_infrared_on();    //������࣬�˳�����û���õ���࣬����û�д򿪲��
/*�رշ�����ȴ���ʼ����*/
	FunVelocity_Control(max_velocity, min_velocity,1000);//�رշ��
	
/******************************���׳�ʼ��********************************************************************/
//��ת����ת�����#########################################################################################
//����������������#########################################################################################
	canbuf[0]=0x03;
	canbuf[1]=0x01;
	CAN1_Send_Msg(canbuf,2,0x301);
	delay_ms(1);
//���Ӽ�ȡ���׼н�#########################################################################################
	canbuf[0]=0x08;
	canbuf[1]=0x00;
	CAN1_Send_Msg(canbuf,2,0x301);
	delay_ms(1);
	delay_ms(100);
	canbuf[1]=0x01;
	CAN1_Send_Msg(canbuf,2,0x301);
	delay_ms(1);
//������������#############################################################################################
	canbuf[0]=0x06;
	canbuf[1]=0x01;
	CAN1_Send_Msg(canbuf,2,0x301);
	delay_ms(1);
//�����ȡ���״�#############################################################################################
	canbuf[0]=0x02;
	canbuf[1]=0x01;
	CAN1_Send_Msg(canbuf,2,0x301);
	delay_ms(1);
	canbuf[0]=0x04;
	canbuf[1]=0x00;
	CAN1_Send_Msg(canbuf,2,0x301);
	delay_ms(1);
//����С�������###########################################################################################
	canbuf[0]=0x05;
	canbuf[1]=0x01;
	CAN1_Send_Msg(canbuf,2,0x301);
	delay_ms(1);
//������������#############################################################################################
	canbuf[0]=0x01;
	canbuf[1]=0x01;
	CAN1_Send_Msg(canbuf,2,0x301);
	delay_ms(1);
	
/******�ֶ���������********/	
//���Ӽ�ȡ���׼н�#########################################################################################
	canbuf[0]=0x08;
	canbuf[1]=0x00;
	CAN1_Send_Msg(canbuf,2,0x301);
	delay_ms(100);
	canbuf[1]=0x01;
	CAN1_Send_Msg(canbuf,2,0x301);
	delay_ms(1);
//���Ӽ�ȡ���״�##########################################################################################
	canbuf[0]=0x07;
	canbuf[1]=0x00;
	CAN1_Send_Msg(canbuf,2,0x301);
	delay_ms(100);
	canbuf[1]=0x01;
	CAN1_Send_Msg(canbuf,2,0x301);
	delay_ms(1);
	delay_ms(5000);//������ʱ���������
//���Ӽ�ȡ���׼н�#########################################################################################
	canbuf[0]=0x08;
	canbuf[1]=0x00;
	CAN1_Send_Msg(canbuf,2,0x301);
	delay_ms(100);
	canbuf[1]=0x01;
	CAN1_Send_Msg(canbuf,2,0x301);
	delay_ms(5000);         //�ȴ�ȫ����λģ���ʼ�����  
	set_yuntai_angle(-20);   //��ʼ����ɺ���̨��ת
/*******************************���ң��������**************************************************************/
	float vm = 0;       //�����ã�������FAST_EST_();
	float fast_K;       //�����ã�������FAST_EST_();
	dbus.rc.ch0 = dbus.rc.ch1 = dbus.rc.ch2 = dbus.rc.ch3 = 1024;//��ģ�������м�ֵ����ֹң����û�з�������ʧ��
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
	
/******************************************��ת����ʮ�ȵȴ���ʼ*********************************************/
  k = 1;
	PID_set(&link_pid,0.5,0,0,100,1);
	PID_set(&angle_pid,10,0,0,70,1);
	aim_set(&aim[1],0,-100,-32);
	
	while(dbus.rc.s1!=1);//����࿪�����ϲ�������ת31��,������ǰ��0.1m
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
	
/************************************׼����ʼ**********************************************/
	k = 1;                  //k��һ����
	k_max = 9;              //�����ܵ㣬k_max�����һ�������ż�һ
	int flag2 = 0;          //����ĳ�������־

/*****************************************����PID����*******************************************************/
  PID_set(&link_pid,1.5,0,0,1000,1);      //ǰ����PID
  PID_set(&angle_pid,15,0,0,400,1);     //�Ƕȵ�PID
/****************************************ȷ���н���·��*****************************************************/
  
	aim_set(&aim[1],700,-620,-31);        //��ֱ�ߣ���������
	aim_set(&aim[2],4900,-725,0);         //�����ҹ�ɭ��
	aim_set(&aim[3],5900,-780,0);         //׼�����Ž׶�
	aim_set(&aim[4],7250,-725,0);         //���ţ�7250������Ҫ����
	aim_set(&aim[5],7900,-1500,-90);      //ת��
  aim_set(&aim[6],7850,-2800,-117);	    //׼������
	aim_set(&aim[7],7750,-4400,-118);	    //׼������
	aim_set(&aim[8],7100,-7500,-118);	    //��ǽ�׶Σ��˳�������ǽ
/*����ǽ���ͣ��ó�����ǰ����ʻ��ͬʱ�Ƕ��Դ���-120�ȣ�
  �˳����ó�����(7100,-7000)�ķ���ǰ������ʵ��ͣ����Y������-5000���ң�
	����ͣ���������������void Link_Output(PID_Parameter *pid)*/
	
/**********************************************�ȴ�ң�أ��������*******************************************/
  while(dbus.rc.s1!=3);//����࿪�ز����м�λ�ã���ʼ����
	dbus.rc.ch0 = dbus.rc.ch1 = dbus.rc.ch2 = dbus.rc.ch3 = 1024;//��ģ�������м�ֵ����ֹң����û�з�������ʧ��
	FunVelocity_Control(max_velocity, min_velocity,1400);             //�����
	delay_ms(3000);//��ʱ�ǰ�ȫ���ѣ�����ʱɾ��
	set_yuntai_angle(0);
	
/****************************************��������ת��*****************************************************/	
	while(1)
	{
		if(flag1)
		{
      if(k==5)/******ת�䲿��********/
			{
				turn_v.v1 = turn_v.v2 = turn_v.v3 = 0;
				link_velocity(&link_pid,0); 
				velocity_calcu(); 
				velocity_calcu2();    //������ǰ�����ٶ��м�����ת���ٶ�
				FAST_EST_();          //�ٶȵ������������ٶ�Ϊ1280
				CMControlLoop();      //ʹ��CAN���߷��͵���ٶȿ����ź�
				flag1 = 0;
			}
			else
			{
				Link_Output(&link_pid);      //�������PID��OUTPUT����Ҫ����ͣ��ʱ���٣���ͣ��ʱ����ǰ����ʵ�ʵ�OUTPUTֻ���ṩһ����ת���ٶȱ�����ϵ��������ֵ�����岻��
				if(k==2)                     //�����ң���ɭ��
				{
					link_velocity(&link_pid,angle_diff2_direction());   //������ǰ��������ٶȷ���
					turn_velocity(&angle_pid,angle_diff2_angle());      //�����ҳ�ͷ����ת���ٶȷ���
				}
/*������ʱ��ת�ٶȷ��������Һ����ͺ���ǰ�������ٶȺ�����Լ20cm��λ�Ŀ���Ǳ�֤ת��ļ�ʱ�ԣ��˾����ǲ��Եó�������Ȥ��ͬѧ����ȥ����λ���һ��*/
				else
				{
					link_velocity(&link_pid,angle_diff1());    //���������ֱ�ߣ�ǰ��������ٶȷ���
				  turn_velocity(&angle_pid,aim[k].angle);    //��ͷת���ٶ�
				}
				velocity_calcu();        //���������ٶȼӺ�
				if(k==3)
					adjust_bridge(1.0,400);
				if(k==4)
					adjust_bridge(0.5,300);
				FAST_EST_();             //���ٶȰ��������е���
				CMControlLoop();         //������� 
				flag1 = 0;
			}
		}
		
/*****************************************�ж��Ƿ񵽴�λ��*****************************************************/
		if(k==5)//�ж�ת���Ƿ����
		{
			if((aim[k].y-posi.y)>-200)  //�ﵽ�����ж�
				flag2 = 1;
			if(posi.angle<-60)       //����Ƕ��ж�
				flag2 = 1;
		}
		else if(k<5)//ת��֮ǰ���ж�
		{
			if((aim[k].x-posi.x)<50)  //����֮ǰ�ĵ���Ŀ���ж���x����ֵ
			flag2 = 1;
		}
		else if(k==8)         //ͣ������y�����ж�
		{
			if(posi.y<-5000)    //-5200�д�����������ֵ��void Link_Output(PID_Parameter *pid)�е�2290���Ӧ
			  flag2 = 1;
		}
		else
		{
		  if((aim[k].y-posi.y)>-50)   //����֮��ĵ���Ŀ���ж���y����ֵ
			flag2 = 1;				
		}	
/*********************************************���������ĵ���******************************************************/		
		if(flag2)
		{
			flag2 = 0;
			k++;
			
			if(k==2)
			  PID_set(&angle_pid,35,0,0,200,1);     //�Ƕȵ�PID��������ʱ��һ���ϴ��P��OUTPUT_MAXֵ
			if(k==3)
			{
				PID_set(&angle_pid,20,0,0,300,1);     //�Ƕȵ�PID����Բ��ʱ��P��OUTPUT_MAXֵ����̫��
				FunVelocity_Control(max_velocity, min_velocity,1500);//�Ӵ��������Ŷ�
			}
			if(k==4)
				PID_set(&angle_pid,15,0,0,200,1);     //�Ƕȵ�PID�����ź�
			if(k==6)
				PID_set(&angle_pid,10,0,0,170,1);     //�Ƕȵ�PID��ת��󣬸�һ���Ƚ�С��P��OUTPUT_MAXֵ��ֹת��ײǽ
			if(k==7)
			{
				PID_set(&angle_pid,10,0,0,200,1);     //�Ƕȵ�PID��׼������
				FunVelocity_Control(max_velocity, min_velocity,1000);//�ط��
			}
			if(k==8)
			{
				PID_set(&angle_pid,35,0,0,350,1);     //�Ƕȵ�PID����ǽ��PID�������ϴ��Pֵ��ʹ�Ƕ��ܹ��ȶ����֣����ȶ���ǽ
/****************������ƣ���������*****************************************************************************/
//���������������#############################################################################################
//��ת����ת����ʮ��###########################################################################################
				canbuf[0]=0x03;
				canbuf[1]=0x00;
				CAN1_Send_Msg(canbuf,2,0x301);
				delay_ms(1);
//���������½�#################################################################################################
				canbuf[0]=0x06;
				canbuf[1]=0x00;
				CAN1_Send_Msg(canbuf,2,0x301);
				delay_ms(1);
			}
			if(k>=k_max)
			{
				FunVelocity_Control(max_velocity, min_velocity,1000);//�ط��
				break;
			}
		}
	}
	
	FunVelocity_Control(max_velocity, min_velocity,1000);//�ط��
	Stop();
	
	
	delay_ms(3000);//��ͣ��Ч��������ɾ�� 
	
//��ת����ת�����#########################################################################################
//����������������#########################################################################################
	canbuf[0]=0x03;
	canbuf[1]=0x01;
	CAN1_Send_Msg(canbuf,2,0x301);
	delay_ms(1);
	
	
/*********************************************ץȡ����չ�*********************************************************/
	Stop();
	
	k=1;
	k_max=9;
	int count_flag1 = 0;//ȫ����λģ��ÿ5MS����һ�����ݣ�ͨ������ȫ����λģ�鷵�����ݵĴ��������д��Լ�ʱ����СCPU��ʱѹ��
	int flag_k7 = 1; //k=7y���������ɱ�־
	flag_round_two = 1;//�ڶ��غ�
	
	PID_set(&link_pid,2,0,0,300,0);//�ȴ����������ɽ����ڼ��ٶȲ��úܿ�
	PID_set(&angle_pid,15,0,0,100,1);
	
	aim_set(&aim[1],8500,-4700,90);//׼��
	aim_set(&aim[2],8500,-4500,90);//��һ�μ�ȡ
	aim_set(&aim[3],7800,-3400,180);//����ȴ���
	aim_set(&aim[4],4000,-3200,180);//���뷢����
	aim_set(&aim[5],8100,-3200,180);//�ڶ��μ�ȡ
  aim_set(&aim[6],4000,-3200,180);//�ڶ��ν��뷢����
	aim_set(&aim[7],8100,-2800,180);//�����μ�ȡ
	aim_set(&aim[8],4000,-3200,180);//�����ν��뷢����
	
	
	while(1)
	{
    if(flag1)
		{
			flag1=0;
			Link_Output(&link_pid);
			link_velocity(&link_pid,angle_diff1());//��ֱ��
			turn_velocity(&angle_pid,aim[k].angle);
			velocity_calcu();
			if(k==7&&flag_k7)
			  adjust_y(); //������ץȡʱy�������
			FAST_EST_();
			CMControlLoop();
			
			if(k==4) //��ֹ����̧����ȡ
			{
				count_flag1++;
				if(count_flag1==100)
					set_capture_angle(0);
			}
			
			if(k==5||k==7) //��ץȡ��ǰ���Ĺ����з���ץȡװ��
			{
				count_flag1++;
				if(count_flag1==300)
					set_capture_angle(140);
			}
			
			if(k==6||k==8)   //k=6��ʱ��һ��ǰ����һ�߽������ײ�����ʡʱ��
			{
				count_flag1++;
				if(count_flag1==60)//0.3Sʱ��ȡװ��̧��
					set_capture_angle(0);
				if(count_flag1==300)//1500mSʱת����̨
				{
					set_yuntai_angle(-45);
//����С��������###########################################################################################
					canbuf[0]=0x05;
					canbuf[1]=0x00;
					CAN1_Send_Msg(canbuf,2,0x301);
					delay_ms(1);
				}
				if(count_flag1==400)//2S�򿪷����ȡ������
				{
//�����ȡ���״�#############################################################################################
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

/*************************************����Ŀ����ж�*******************************************************/
		if(abs(aim[k].y-posi.y)<15&&k==7)
			flag_k7 = 0;
		
/*С����ȡ�չǵ�ʱ����ܻ���ֱ��չǿ�ס���޷�����Ŀ�����������ʱ��ң�����������������ϣ�ͬʱ�������Ҳ�ҡ�ˣ����ɽ���ң��״̬*/
		if((k==2||k==5||k==7)&&dbus.rc.s1==1&&dbus.rc.s2==1&&dbus.rc.ch1>1600)
			flag2 = 1;
		
		if(abs(aim[k].x-posi.x)<15&&abs(aim[k].y-posi.y)<15&&abs(aim[k].angle-posi.angle)<2)
			flag2=1;

/**************************************����Ŀ����ĵ���**************************************************/
		if(flag2)
		{
			flag2=0;
			k++;
			
			if(k==2)//��һ�μ�ȡ
				set_capture_angle(140);//���¼�ȡװ�ã���û����ȫ����
			
			if(k==3||k==6||k==8)
			{
				Stop();
				count_flag1 = 0;//��������
				set_capture_angle(160);//���¼�ȡװ��
//�����ȡ���״�#############################################################################################
				canbuf[0]=0x02;
				canbuf[1]=0x01;
				CAN1_Send_Msg(canbuf,2,0x301);
				delay_ms(1);
				canbuf[0]=0x04;
				canbuf[1]=0x00;
				CAN1_Send_Msg(canbuf,2,0x301);
				delay_ms(1000);
/**************************************�ֶ���ȡ*********************************************************
��1��һ��ÿ�ε����ȡ��ʱ����ȡ���׼н���С���Զ�����ң��ģʽ��
��2��S1���ϲ���ץȡװ�÷��£����²�ץȡװ������
��3��S2�����Ƽ�ȡ�չ����״򿪣������Ƽ�ȡ�չ����׼н���
��3����ҡ�˿���ǰ������ǰ����ǰ��������Ե��ӣ���ҡ�˿���ת��
��4��S1��S2�������м�λ��ʱ�����ұߵ�ҡ�������ƣ�С�������Զ�ģʽ��*/
/**��������������������������������������������������������������������������������������������������������������������������������������������������������������������������������������������������**/
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
					
					if(dbus.rc.s1==1&&m3508_capture_angle!=160)//��߿�����������ץȡװ�÷���
						set_capture_angle(160);
					if(dbus.rc.s1==2&&m3508_capture_angle!=0)//��߿�����������ץȡװ������
						set_capture_angle(0);
					
					if(dbus.rc.s2==1&&canbuf[0]!=0x04)//�ұ߿�������������ȡ���״�
					{
//�����ȡ���״�#############################################################################################
						canbuf[0]=0x02;
						canbuf[1]=0x01;
						CAN1_Send_Msg(canbuf,2,0x301);
						delay_ms(1);
						canbuf[0]=0x04;
						canbuf[1]=0x00;
						CAN1_Send_Msg(canbuf,2,0x301);
						delay_ms(1);
					}
					if(dbus.rc.s2==2&&canbuf[0]!=0x02)//�ұ߿�������������ȡ��������
					{
//�����ȡ���׼н�#############################################################################################
						canbuf[0]=0x04;
						canbuf[1]=0x01;
						CAN1_Send_Msg(canbuf,2,0x301);
						delay_ms(1);
						canbuf[0]=0x02;
						canbuf[1]=0x00;
						CAN1_Send_Msg(canbuf,2,0x301);
						delay_ms(1);
					}
					
					if(dbus.rc.s1==3&&dbus.rc.s2==3&&dbus.rc.ch1>1600)//�������ض������м�λ�ã�Ȼ����ǰ���Ҳ�ҡ�ˣ�С�������Զ�ģʽ
						break;
				}
/**��������������������������������������������������������������������������������������������������������������������������������������������������������������������������������������������������**/
			
//�����ȡ���׼н�#############################################################################################
				canbuf[0]=0x04;
				canbuf[1]=0x01;
				CAN1_Send_Msg(canbuf,2,0x301);
				delay_ms(1);
				canbuf[0]=0x02;
				canbuf[1]=0x00;
				CAN1_Send_Msg(canbuf,2,0x301);
				delay_ms(1);
			}
			
			if(k==4)//���뷢����
			{
				Stop();
				count_flag1 = 0;//��������
				set_yuntai_angle(-45);//��̨�Ƕ�
				PID_set(&link_pid,1,0,0,500,0);   //���뷢�������ٶ�
	      PID_set(&angle_pid,15,0,0,200,1);   //���뷢�����ٶ�
//����С��������###############################################################################################
				canbuf[0]=0x05;
				canbuf[1]=0x00;
				CAN1_Send_Msg(canbuf,2,0x301);
				delay_ms(1000);
//�����ȡ���״�#############################################################################################
				canbuf[0]=0x02;
				canbuf[1]=0x01;
				CAN1_Send_Msg(canbuf,2,0x301);
				delay_ms(1);
				canbuf[0]=0x04;
				canbuf[1]=0x00;
				CAN1_Send_Msg(canbuf,2,0x301);
				while(dbus.rc.s1==2||dbus.rc.s1==3);//��߿��������������뷢����
			}
			if(k==5||k==7||k==9)//����
			{
				Stop();
        count_flag1 = 0;//��������
//�����ȡ���״�#############################################################################################
				canbuf[0]=0x02;
				canbuf[1]=0x01;
				CAN1_Send_Msg(canbuf,2,0x301);
				delay_ms(1);
				canbuf[0]=0x04;
				canbuf[1]=0x00;
				CAN1_Send_Msg(canbuf,2,0x301);
				delay_ms(200);//���ϼ�ȡ���״���ʵ�ʲ��Ժ��ɾ��
//���������Ƴ�#################################################################################################
				canbuf[0]=0x01;
				canbuf[1]=0x00;
				CAN1_Send_Msg(canbuf,2,0x301);
				delay_ms(2000);//��������ʱ��
//������������#################################################################################################
				canbuf[0]=0x01;
				canbuf[1]=0x01;
				CAN1_Send_Msg(canbuf,2,0x301);
				delay_ms(1);
//����С�������###############################################################################################
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
	
	FunVelocity_Control(max_velocity, min_velocity,1000);//�ط��
	Stop();//���ֲ���
	
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


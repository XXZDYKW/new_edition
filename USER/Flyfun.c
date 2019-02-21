#include "Flyfun.h"
#include "delay.h"

/*
 * @brief：TIM9_CH1的PWM初始化
 * @attention：脉冲周期T = (arr+1)*(psc+1)/84M 
 * @param：arr(自动重装值)；psc：时钟预分频数
 * @retval：none
*/
void TIM9_PWM_Init(u32 arr,u32 psc)
{		 					 
	
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9,ENABLE);  	   
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE); 		
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource5,GPIO_AF_TIM9); 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;           
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;    
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        
	GPIO_Init(GPIOE,&GPIO_InitStructure);         
	  
	TIM_TimeBaseStructure.TIM_Prescaler=psc;  
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; 
	TIM_TimeBaseStructure.TIM_Period=arr;   
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	TIM_TimeBaseInit(TIM9,&TIM_TimeBaseStructure);
	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; 
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; 
	TIM_OC1Init(TIM9, &TIM_OCInitStructure);  

	TIM_OC1PreloadConfig(TIM9, TIM_OCPreload_Enable);  
	TIM_ARRPreloadConfig(TIM9,ENABLE);
	TIM_Cmd(TIM9, ENABLE);  						  
}  

/*
 * @brief：风机PWM初始化
 * @attention：none
 * @param：none
 * @retval：none
*/
void Fun_Init(void)
{
	TIM9_PWM_Init(20000-1,168-1);
	delay_ms(500);
}


/*
 * @brief 风机速度控制
 * @attention：none
* @param： max_velocity(最大速度)；max_velocity(最小速度)；aim_velocity(目标速度)
* @retval：none
*/
void FunVelocity_Control(int max_velocity,int min_velocity,int aim_velocity)
{
	if(aim_velocity > max_velocity) 	aim_velocity=max_velocity;
	if(aim_velocity < min_velocity)		aim_velocity=min_velocity;
	TIM_SetCompare1(TIM9,aim_velocity);	
}

























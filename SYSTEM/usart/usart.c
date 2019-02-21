#include "sys.h"
#include "usart.h"	
#include "main.h"

#if SYSTEM_SUPPORT_OS
#include "includes.h"					//ucos 使用	  
#endif


#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{ 	
	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
	USART1->DR = (u8) ch;      
	return ch;
}
#endif

int flag1;                               //串口2接收数据完成标志

/*****************串口1INITIAL**********************/
void USART1_Init(int baudrate)
{
	GPIO_InitTypeDef     GPIO_InitStructure; 
	USART_InitTypeDef    USART_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);    //gpio时钟使能
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);   //串口时钟使能
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); //端口复用
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); //端口复用
	
	//USART1端口配置
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;  //引脚复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;  //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	//上拉模式
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	//USART1 初始化设置
	USART_InitStructure.USART_BaudRate = baudrate;  //波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;  //字长
	USART_InitStructure.USART_StopBits = USART_StopBits_1;  //停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;  //奇偶校验
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;//收发模式
	USART_Init(USART1, &USART_InitStructure);//串口初始化
	
	USART_Cmd(USART1, ENABLE);//串口1使能

}

/******************串口2初始化函数********************/
void USART2_Init(int baudrate)
{
	GPIO_InitTypeDef     GPIO_InitStructure; 
	USART_InitTypeDef    USART_InitStructure;
	NVIC_InitTypeDef     NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);        //gpio时钟使能
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);       //串口时钟使能
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2);     //端口复用
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2);     //端口复用
	
	//USART2端口配置
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;      //GPIO2和GPIO3
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;                //引脚复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;              //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	              //上拉模式
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	//USART2 初始化设置
	USART_InitStructure.USART_BaudRate = baudrate;              //波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b; //字长
	USART_InitStructure.USART_StopBits = USART_StopBits_1;      //停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;         //奇偶校验
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;                //收发模式
	USART_Init(USART2, &USART_InitStructure);                   //串口初始化
	
	USART_Cmd(USART2, ENABLE);                                  //串口2使能

	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	
	//USART1 NVIC 中断设置
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;          //串口2中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;    //抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;          //响应优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;            //通道使能
	NVIC_Init(&NVIC_InitStructure);
}  


/*串口2中断函数*/
/*读取定位模块的坐标和角度数据*/


position posi;      //define position type posi

void USART2_IRQHandler(void)
{
	
	static uint16_t count=0;
	static uint8_t last=0;
	static uint8_t ch;
	static uint8_t flag=0;
	static union
	{
		uint8_t data[24];
		float ActVal[6];
	}posture;
	
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
		{
			USART_ClearITPendingBit( USART2, USART_FLAG_RXNE); 
			ch=USART_ReceiveData(USART2);   //读取接收到的数据	
			if(flag==1)
			{
				posture.data[count]=ch;
				count++;
				if(count>=24)
				{
					count=0;
					flag=0;
					posi.angle=posture.ActVal[0];  //angle
					posi.y =-1*posture.ActVal[3];     //坐标X
					posi.x =posture.ActVal[4];     //坐标Y
					posi.x =posi.x-260+260*cos(posi.angle/57.2958);
	        posi.y =posi.y+260*sin(posi.angle/57.2958);//此处是加，前面的版本可能有错误
					flag1=1;
				}
			}
			else 
			{
				if(last==0x0d&&ch==0x0a)
			  {
				  flag=1;
			  }
			  last=ch;
			}
		}
}	

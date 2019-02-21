#include "sys.h"
#include "usart.h"	
#include "main.h"

#if SYSTEM_SUPPORT_OS
#include "includes.h"					//ucos ʹ��	  
#endif


#if 1
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//�ض���fputc���� 
int fputc(int ch, FILE *f)
{ 	
	while((USART1->SR&0X40)==0);//ѭ������,ֱ���������   
	USART1->DR = (u8) ch;      
	return ch;
}
#endif

int flag1;                               //����2����������ɱ�־

/*****************����1INITIAL**********************/
void USART1_Init(int baudrate)
{
	GPIO_InitTypeDef     GPIO_InitStructure; 
	USART_InitTypeDef    USART_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);    //gpioʱ��ʹ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);   //����ʱ��ʹ��
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); //�˿ڸ���
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); //�˿ڸ���
	
	//USART1�˿�����
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;  //���Ÿ��ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;  //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	//����ģʽ
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	//USART1 ��ʼ������
	USART_InitStructure.USART_BaudRate = baudrate;  //������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;  //�ֳ�
	USART_InitStructure.USART_StopBits = USART_StopBits_1;  //ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;  //��żУ��
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;//�շ�ģʽ
	USART_Init(USART1, &USART_InitStructure);//���ڳ�ʼ��
	
	USART_Cmd(USART1, ENABLE);//����1ʹ��

}

/******************����2��ʼ������********************/
void USART2_Init(int baudrate)
{
	GPIO_InitTypeDef     GPIO_InitStructure; 
	USART_InitTypeDef    USART_InitStructure;
	NVIC_InitTypeDef     NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);        //gpioʱ��ʹ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);       //����ʱ��ʹ��
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2);     //�˿ڸ���
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2);     //�˿ڸ���
	
	//USART2�˿�����
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;      //GPIO2��GPIO3
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;                //���Ÿ��ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;              //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	              //����ģʽ
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	//USART2 ��ʼ������
	USART_InitStructure.USART_BaudRate = baudrate;              //������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b; //�ֳ�
	USART_InitStructure.USART_StopBits = USART_StopBits_1;      //ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;         //��żУ��
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;                //�շ�ģʽ
	USART_Init(USART2, &USART_InitStructure);                   //���ڳ�ʼ��
	
	USART_Cmd(USART2, ENABLE);                                  //����2ʹ��

	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	
	//USART1 NVIC �ж�����
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;          //����2�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;    //��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;          //��Ӧ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;            //ͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);
}  


/*����2�жϺ���*/
/*��ȡ��λģ�������ͽǶ�����*/


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
			ch=USART_ReceiveData(USART2);   //��ȡ���յ�������	
			if(flag==1)
			{
				posture.data[count]=ch;
				count++;
				if(count>=24)
				{
					count=0;
					flag=0;
					posi.angle=posture.ActVal[0];  //angle
					posi.y =-1*posture.ActVal[3];     //����X
					posi.x =posture.ActVal[4];     //����Y
					posi.x =posi.x-260+260*cos(posi.angle/57.2958);
	        posi.y =posi.y+260*sin(posi.angle/57.2958);//�˴��Ǽӣ�ǰ��İ汾�����д���
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

#include "main.h"
#include "CAN.h"
unsigned int CAN_Time_Out = 0;
unsigned char can_tx_success_flag = 0;
void CAN_Delay_Us(unsigned int t)
{
	int i;
	for(i=0;i<t;i++)
	{
		int a=40;
		while(a--);
	}
}


/***********can初始化函数**************/
void My_can_init(void)
{
	CAN1_Configuration();                                //1M，不是500K
	delay_ms(500);                                       //刚开始要有足够的延时，确保驱动器已经初始化完成  

}

/*----CAN1_TX-----PA12----*/
/*----CAN1_RX-----PA11----*/

/*************************************************************************
                          CAN1_Configuration
描述：初始化CAN1配置为1M波特率
*************************************************************************/
void CAN1_Configuration(void)
{
    CAN_InitTypeDef        can;
    CAN_FilterInitTypeDef  can_filter;
    GPIO_InitTypeDef       gpio;
    NVIC_InitTypeDef       nvic;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_CAN1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_CAN1);

    gpio.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_11;
    gpio.GPIO_Mode = GPIO_Mode_AF;
	  gpio.GPIO_OType = GPIO_OType_PP;//推挽输出新加
    gpio.GPIO_Speed = GPIO_Speed_100MHz;//100MHz新加
    gpio.GPIO_PuPd = GPIO_PuPd_UP;//上拉    新加
    GPIO_Init(GPIOA, &gpio);
    
    nvic.NVIC_IRQChannel = CAN1_TX_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;  //1
    nvic.NVIC_IRQChannelSubPriority = 1;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
    
    CAN_DeInit(CAN1);
    CAN_StructInit(&can);
    
    can.CAN_TTCM = DISABLE;
    can.CAN_ABOM = DISABLE;
    can.CAN_AWUM = DISABLE;
    can.CAN_NART = DISABLE;
    can.CAN_RFLM = DISABLE;
    can.CAN_TXFP = ENABLE;
    can.CAN_Mode = CAN_Mode_Normal;
    can.CAN_SJW  = CAN_SJW_1tq;
    can.CAN_BS1 = CAN_BS1_9tq;
    can.CAN_BS2 = CAN_BS2_4tq;
    can.CAN_Prescaler = 3;   //CAN BaudRate 42/(1+9+4)/3=1Mbps
    CAN_Init(CAN1, &can);

	can_filter.CAN_FilterNumber = 0;
	can_filter.CAN_FilterMode = CAN_FilterMode_IdMask;
	can_filter.CAN_FilterScale = CAN_FilterScale_32bit;
	can_filter.CAN_FilterIdHigh = 0x0000;
	can_filter.CAN_FilterIdLow = 0x0000;
	can_filter.CAN_FilterMaskIdHigh = 0x0000;
	can_filter.CAN_FilterMaskIdLow = 0x0000;
	can_filter.CAN_FilterFIFOAssignment = 0;
	can_filter.CAN_FilterActivation=ENABLE;
	CAN_FilterInit(&can_filter);
    
    CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);
    CAN_ITConfig(CAN1,CAN_IT_TME,ENABLE);
}

/*************************************************************************
                          CAN1_TX_IRQHandler
描述：CAN1的发送中断函数
*************************************************************************/
void CAN1_TX_IRQHandler(void)
{
    if (CAN_GetITStatus(CAN1,CAN_IT_TME)!= RESET) 
	{
	   CAN_ClearITPendingBit(CAN1,CAN_IT_TME);
     can_tx_success_flag=1;
  }
}




uint8_t status;
uint8_t TransmitMailbox;
int cnt;
unsigned int CAN_Time_Out;
unsigned char can_tx_success_flag;

u8 CAN1_Send_Msg(u8* msg,u8 len,u16 ID)
{	
  u8 mbox;
  u16 i=0;
  CanTxMsg TxMessage;
  TxMessage.StdId=ID;	 // 标准标识符为0
  TxMessage.ExtId=0x00;	 // 设置扩展标示符（29位）
  TxMessage.IDE=0;		  // 使用扩展标识符
  TxMessage.RTR=0;		  // 消息类型为数据帧，一帧8位
  TxMessage.DLC=len;							 // 发送两帧信息
  for(i=0;i<len;i++)
  TxMessage.Data[i]=msg[i];				 // 第一帧信息          
  mbox= CAN_Transmit(CAN1, &TxMessage);   
  i=0;
  while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//等待发送结束
  if(i>=0XFFF)return 1;
  return 0;		
}

void CAN_Send_Msg1(short Velocity_M1,short Velocity_M2,short Velocity_M3)
{
  
	CanTxMsg tx_message;
	tx_message.StdId = 0x12;
	tx_message.IDE = CAN_ID_STD;
	tx_message.RTR = CAN_RTR_DATA;
	tx_message.DLC = 0x08;
  
	
	tx_message.Data[0] = (u8)(Velocity_M1>>8);
	tx_message.Data[1] = (u8)Velocity_M1;
	tx_message.Data[2] = (u8)(Velocity_M2>>8);
	tx_message.Data[3] = (u8)Velocity_M2;
	tx_message.Data[4] = (u8)(Velocity_M3>>8);
	tx_message.Data[5] = (u8)Velocity_M3;
  
	can_tx_success_flag = 0;
	cnt=0;
	
	CAN_Transmit(CAN1,&tx_message);
    CAN_Time_Out = 0;
	while(can_tx_success_flag == 0)
    {
			delay_us(10);
			CAN_Time_Out++;
			if(CAN_Time_Out>100)	break;
    }
}

extern short m3508_capture_angle;
extern short m2006_yuntai_angle;

void CAN_Send_Msg2()
{
  
	CanTxMsg tx_message;
	tx_message.StdId = 0x13;
	tx_message.IDE = CAN_ID_STD;
	tx_message.RTR = CAN_RTR_DATA;
	tx_message.DLC = 0x08;
  
	
	tx_message.Data[0] = (u8)(m3508_capture_angle>>8);
	tx_message.Data[1] = (u8)m3508_capture_angle;
	tx_message.Data[2] = (u8)(m2006_yuntai_angle>>8);
	tx_message.Data[3] = (u8)m2006_yuntai_angle;
  
	can_tx_success_flag = 0;
	cnt=0;
	
	CAN_Transmit(CAN1,&tx_message);
    CAN_Time_Out = 0;
	while(can_tx_success_flag == 0)
    {
			delay_us(10);
			CAN_Time_Out++;
			if(CAN_Time_Out>100)	break;
    }
}

void set_yuntai_angle(short yuntai_angle)
{
	m2006_yuntai_angle = yuntai_angle;
	CAN_Send_Msg2();
}

void set_capture_angle(short capture_angle)
{
	m3508_capture_angle = capture_angle;
	CAN_Send_Msg2();
}





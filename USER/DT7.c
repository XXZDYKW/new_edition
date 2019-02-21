#include "DT7.h"

/*遥控器初始化*/
/*初始化串口1的RX，波特率100kpb，8位，一个停止位，偶校验 */
/*初始化DMA2_CH4_stream5,设置优先级开启中断*/

unsigned char bus_buf[DBUS_BUF_SIZE];

void DT7_init(void)
{
    USART_InitTypeDef usart1;
    GPIO_InitTypeDef  gpio;
    NVIC_InitTypeDef  nvic;
    DMA_InitTypeDef   dma;
    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_DMA2,ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
    
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource10 ,GPIO_AF_USART1);
    
    gpio.GPIO_Pin = GPIO_Pin_10 ;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA,&gpio);
    
    USART_DeInit(USART1);
    usart1.USART_BaudRate = 100000;   //SBUS 100K baudrate
    usart1.USART_WordLength = USART_WordLength_8b;
    usart1.USART_StopBits = USART_StopBits_1;
    usart1.USART_Parity = USART_Parity_Even;
    usart1.USART_Mode = USART_Mode_Rx;
    usart1.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART1,&usart1);
    
    USART_Cmd(USART1,ENABLE);
    USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);
    
    nvic.NVIC_IRQChannel = DMA2_Stream5_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority = 1;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
    
    DMA_DeInit(DMA2_Stream5);
    dma.DMA_Channel= DMA_Channel_4;
    dma.DMA_PeripheralBaseAddr = (u32)&(USART1->DR);
    dma.DMA_Memory0BaseAddr = (u32)bus_buf;
    dma.DMA_DIR = DMA_DIR_PeripheralToMemory;
    dma.DMA_BufferSize = DBUS_BUF_SIZE;
    dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    dma.DMA_Mode = DMA_Mode_Circular;
    dma.DMA_Priority = DMA_Priority_VeryHigh;
    dma.DMA_FIFOMode = DMA_FIFOMode_Disable;
    dma.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
    dma.DMA_MemoryBurst = DMA_Mode_Normal;
    dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA2_Stream5,&dma);

    DMA_ITConfig(DMA2_Stream5,DMA_IT_TC,ENABLE);
    DMA_Cmd(DMA2_Stream5,ENABLE);
}


/*DT7的通道数据转换*/
void DBUS_Dec(DBUS* dbus,const unsigned char* buf)
{
    dbus->rc.ch0 = (buf[0] | (buf[1] << 8)) & 0x07ff;          //!< Channel 0  
    dbus->rc.ch1 = ((buf[1] >> 3) | (buf[2] << 5)) & 0x07ff;   //!< Channel 1         
    dbus->rc.ch2 = ((buf[2] >> 6) | (buf[3] << 2) | (buf[4] << 10)) & 0x07ff;           //!< Channel 2                          
    dbus->rc.ch3 = ((buf[4] >> 1) | (buf[5] << 7)) & 0x07ff;   //!< Channel 3   
    dbus->rc.s1 = ((buf[5] >> 4) & 0x000C) >> 2;                    //!< Switch left         
    dbus->rc.s2 = ((buf[5] >> 4) & 0x0003);                         //!< Switch right  
    dbus->mouse.x = buf[6] | (buf[7] << 8);                    //!< Mouse X axis 
    dbus->mouse.y = buf[8] | (buf[9] << 8);                    //!< Mouse Y axis 
    dbus->mouse.z = buf[10] | (buf[11] << 8);                  //!< Mouse Z axis 
    dbus->mouse.l = buf[12];                                        //!< Mouse Left Is Press ?
    dbus->mouse.r = buf[13];                                        //!< Mouse Right Is Press ? 
    dbus->key.v = buf[14] | (buf[15] << 8);                    //!< KeyBoard value   
    dbus->res = buf[16] | (buf[17] << 8);                      //!< Reserve 
}

/*DMA2中断*/
DBUS dbus;
void DMA2_Stream5_IRQHandler(void)
{
    if(DMA_GetITStatus(DMA2_Stream5, DMA_IT_TCIF5))
    {
       
        DMA_ClearFlag(DMA2_Stream5, DMA_FLAG_TCIF5);
        DMA_ClearITPendingBit(DMA2_Stream5, DMA_IT_TCIF5);
		
        DBUS_Dec(&dbus,bus_buf);//接收遥控数据
        
    }
}



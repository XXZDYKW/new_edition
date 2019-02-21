#ifndef __USART_H
#define __USART_H
#include "stm32f4xx_conf.h"
#include "main.h"

void USART1_Init(int baudrate);
void USART2_Init(int baudrate);

extern int flag1;

#endif



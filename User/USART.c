#include "stdafx.h"

uint32_t Rx1Counter=0;
uint8_t Rx1Buffer[MAX_BUFFER_SIZE];
uint32_t Rx2Counter=0;
uint32_t CCR_Data;
uint8_t Rx2Buffer[MAX_BUFFER_SIZE];

int fputc(int ch,FILE *f)
{
	USART_SendData(USART1,(unsigned char)ch);
	while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);
	return ch;	
}

void USART1_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING;//test
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_10;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	USART_InitStructure.USART_BaudRate=9600;
	USART_InitStructure.USART_Mode=USART_Mode_Rx|USART_Mode_Tx;
	USART_InitStructure.USART_Parity=USART_Parity_No;
	USART_InitStructure.USART_StopBits=USART_StopBits_1;
	USART_InitStructure.USART_WordLength=USART_WordLength_8b;
	USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
	USART_Init(USART1,&USART_InitStructure);
	
	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);
	
	USART_Cmd(USART1,ENABLE);
}
/*
//串口中断 
void USART1_IRQHandler(void) {
	//处理接收到的数据 
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) 
	{ 
		// Clear the USART1 Receive interrupt
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
	} 
	//发送中断 
	if (USART_GetITStatus(USART1, USART_IT_TXE) != RESET) 
	{  
		USART_SendData(USART1, Send_Data[Send_Length++]); 
		if (Send_Length==SEND_LENGTH) 
		{  
			//发送字符结束 
			USART_ClearITPendingBit(USART1,USART_IT_TXE); 
			USART_ITConfig(USART1, USART_IT_TXE, DISABLE); 
			USART_ITConfig(USART1, USART_IT_TC, ENABLE); 
		}
	}
  //发送完成
	if (USART_GetITStatus(USART1, USART_IT_TC) != RESET) 
	{  
		USART_ClearITPendingBit(USART1,USART_IT_TC); 
		USART_ITConfig(USART1, USART_IT_TC, DISABLE); 
	} 
} 
*/

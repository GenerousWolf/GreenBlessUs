#ifndef __USART_H
#define __USART_H

#include "stdafx.h"

#define MAX_BUFFER_SIZE 255
extern uint32_t Rx1Counter;
extern uint8_t Rx1Buffer[];
extern uint8_t Rx2Buffer[];
extern uint32_t Rx2Counter;
extern uint32_t RxACounter;
extern uint8_t RxABuffer[];
int fputc(int ch,FILE *f);
void USART1_Config(void);
void UART4_Config(void);
void USART3_Config(void);
#endif

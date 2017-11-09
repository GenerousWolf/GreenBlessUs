#ifndef __USART_H
#define __USART_H

#include "stdafx.h"

#define MAX_BUFFER_SIZE 225
extern uint32_t Rx1Counter;
extern uint8_t Rx1Buffer[];
extern uint32_t CCR_Data;
extern uint8_t Rx2Buffer[];
extern uint32_t Rx2Counter;

int fputc(int ch,FILE *f);
void USART1_Config(void);

#endif

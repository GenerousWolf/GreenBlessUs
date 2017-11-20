#ifndef __SYSTICK_H
#define __SYSTICK_H

#include "stdafx.h"

void SysTick_Init(void);
void Delay_10us(uint32_t nTime);
void TimingCnt_Decrement(void);

#endif

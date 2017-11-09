#include "stdafx.h"

uint32_t TimingCnt;

void SysTick_Init(void)
{
	if(SysTick_Config(SystemCoreClock/100000) )
	{
		while(1);
	}
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;
}
void Delay_10us(uint32_t nTime)
{
	TimingCnt=nTime;
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk;
	while(TimingCnt!=0x00);
}
void TimingCnt_Decrement(void)
{
	if(TimingCnt!=0x00)
	{
		TimingCnt--;
	}
}


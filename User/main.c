#include "stdafx.h"

int main()
{	
	//Periphal Clocks Launcher
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4 | RCC_APB1Periph_TIM6, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8 , ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4 | RCC_APB1Periph_USART3, ENABLE);
	
	GatePoint.X = 105;
	GatePoint.Y = 0;
	info.nSelfState = ATK;
	
	TIM8_PWM_Init();					//电机enable pwm输出
	
	SysTick_Init();
	USART1_Config();
	puts("Begin Setup");
	USART3_Config();
	Delay_10us(500000);
	puts("End Setup");
	M_Init();													//电机正负控制
	Encoder_Init();
	TIM6_Init();
	UART4_Config();
	NVIC_Config();

	

	while(1)
	{
		Delay_10us(10000);
	}
}


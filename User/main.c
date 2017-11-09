#include "stdafx.h"

int16_t motorSpeeds[3] = {0,0,0};

void MyRun(void);

int main()
{	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 |
												 RCC_APB1Periph_TIM4 , ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO|RCC_APB2Periph_GPIOA|RCC_APB2Periph_USART1,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8 , ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE);
	
	TIM8_PWM_Init();					//���enable pwm���
	
	SysTick_Init();
	USART1_Config();
	M_Init();													//�����������
	Encoder_Init();
	NVIC_Config();
	/*
	I2C_MPU6050_Init();
	InitMPU6050();
	
	SetupAllPivot();
	*/
	
	while(1)
	{	
		MyRun();
		//GetAllFromMPU();
		//Delay_10us(2000);
	}
}

void MyRun(void)
{
	if(Rx1Counter == 24)
	{
		uint8_t uPos = 0; 
		for(int i = 0; i < 3; ++i)
		{
			if(Rx1Buffer[uPos] == '%' && Rx1Buffer[uPos + 7] == '$' && Rx1Buffer[uPos + 1] == 'M')
			{
				uint8_t uSel = Rx1Buffer[uPos + 2] - '0';
				int8_t nDir;
				switch(Rx1Buffer[uPos + 3])
				{
					case '+':
						nDir = 1;
					break;
					case '-':
						nDir = -1;
					break;
					case '0':
						nDir = 0;
					break;
					default:
						break;
				}
				motorSpeeds[uSel] = nDir * ((Rx1Buffer[uPos + 4] - '0') * 100 + (Rx1Buffer[uPos + 5] - '0') * 10 + (Rx1Buffer[uPos + 6] - '0'));
				uPos += 8;
			}
		}
		Rx1Counter = 0;
		Motor_Speed_Control(motorSpeeds[0],0);
		Motor_Speed_Control(motorSpeeds[1],1);
		Motor_Speed_Control(motorSpeeds[2],2);
	}
}

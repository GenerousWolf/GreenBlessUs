#include "stdafx.h"

int16_t motorSpeeds[3] = {0,0,0};

void DataProc(void);

int main()
{	
	//Periphal Clocks Launcher
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4 | RCC_APB1Periph_TIM6, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8 , ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
	
	TIM8_PWM_Init();					//电机enable pwm输出
	
	SysTick_Init();
	USART1_Config();
	M_Init();													//电机正负控制
	Encoder_Init();
	I2C_MPU6050_Init();
	InitMPU6050();
	puts("Begin Setup");
	SetupAllPivot();
	TIM6_Init();
	NVIC_Config();
	puts("End Setup");
	
	while(1)
	{	
		DataProc();
		Delay_10us(5000);
		move();
	}
}

void DataProc(void)
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
	if(Rx2Counter == 32)
    {
		putchar('b');
		putchar('\n');
		if(Rx2Buffer[0] >= 0xFC && Rx2Buffer[30] == 0x0D && Rx2Buffer[31] == 0x0A)
		{
			info.byteShootOut = Rx2Buffer[0] & 0x02;
			info.byteShootSide = Rx2Buffer[0] & 0x01;
			info.byteMatchStatus = (Rx2Buffer[1] & 0xC0) >> 6;
			info.uTimeByRounds = (((uint16_t)(Rx2Buffer[1] & 0x3F) << 8) | Rx2Buffer[2]);
			info.ptSelf.X = Rx2Buffer[3];
			info.ptSelf.Y = (((uint16_t)Rx2Buffer[4] << 8) | Rx2Buffer[5]);
			info.ptRival.X = Rx2Buffer[6];
			info.ptRival.Y = (((uint16_t)Rx2Buffer[7] << 8) | Rx2Buffer[8]);
			info.ptBall.X = Rx2Buffer[9];
			info.ptBall.Y = (((uint16_t)Rx2Buffer[10] << 8) | Rx2Buffer[11]);
			info.nHaltRoundsSelf = (((uint16_t)Rx2Buffer[12] << 8) | Rx2Buffer[13]);
			info.nHaltRoundsRival = (((uint16_t)Rx2Buffer[14] << 8) | Rx2Buffer[15]);
			info.nEvilSelf = Rx2Buffer[16];
			info.nEvilRival = Rx2Buffer[17];
			info.nScoreSelf = Rx2Buffer[18];
			info.nScoreRival = Rx2Buffer[19];
		}
		addNewPoint(info.ptSelf, info.ptBall);
		printf("ballx:%d\n",info.ptBall.X); 
		Rx2Counter = 0;
		putchar('e');
		putchar('\n');
    }
}

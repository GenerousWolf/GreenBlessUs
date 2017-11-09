#include "stdafx.h"
void TIM8_PWM_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;

	//motor enable pwm output		
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;                
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);	
         
  TIM_TimeBaseStructure.TIM_Period = 900;
  TIM_TimeBaseStructure.TIM_Prescaler = 0;           
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1 ;        
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
  TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);
	

  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;         
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;        
  // TIM_OCInitStructure.TIM_Pulse = 0;       //???  2000是自己加的
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;  
	// TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
	
  TIM_OC1Init(TIM8, &TIM_OCInitStructure);             
  TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);	
	TIM_OC2Init(TIM8, &TIM_OCInitStructure);             
  TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);
	TIM_OC3Init(TIM8, &TIM_OCInitStructure);             
  TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable);	
	TIM_OC4Init(TIM8, &TIM_OCInitStructure);             
  TIM_OC4PreloadConfig(TIM8, TIM_OCPreload_Enable);

  TIM_Cmd(TIM8, ENABLE);  
	TIM_CtrlPWMOutputs(TIM8,ENABLE);
}


//control the plus & minus of motor
void M_Init(void)
{	
	GPIO_InitTypeDef GPIO_InitStructure;
	
	//motor 正负控制
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 |GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}


//TIM8 - output pwm
//PC6 - motor 0
//PC7 - motor 1
//PC8 - motor 2

//TIM2 - motor 0 feedback
//TIM3 - motor 1 feedback
//TIM4 - motor 2 feedback
void Motor_Speed_Control(s16 motorSpeed, u8 wheelNum){
	s16 speed = 0 ;	
	
  if(motorSpeed>100)  speed = 100;
	  else if (motorSpeed<-100)  speed = -100;
			else  speed = motorSpeed;
	if(speed == 0)
	{
		switch(wheelNum)
		{
			case 0:{
				GPIO_ResetBits(GPIOA, GPIO_Pin_2);
				GPIO_ResetBits(GPIOA, GPIO_Pin_3);
				break;
			}
			case 1:{
				GPIO_ResetBits(GPIOB, GPIO_Pin_0);
				GPIO_ResetBits(GPIOB, GPIO_Pin_1);
				break;
			}
			case 2:{
				GPIO_ResetBits(GPIOB, GPIO_Pin_8);
				GPIO_ResetBits(GPIOB, GPIO_Pin_9);
				break;
			}
			default:
				break;
		}
	}
  else if(speed > 0)
	{
		speed = 0.45 * speed + 45;
		switch(wheelNum)
		{
			case 0:
			{
				GPIO_ResetBits(GPIOA, GPIO_Pin_2);
				GPIO_SetBits(GPIOA, GPIO_Pin_3);
				TIM8->CCR1 = speed * 9;
				break;
			}
			case 1:
			{
				GPIO_ResetBits(GPIOB, GPIO_Pin_0);
				GPIO_SetBits(GPIOB, GPIO_Pin_1);
				TIM8->CCR2 = speed * 9;
				break;
			}
			case 2:
			{
				GPIO_ResetBits(GPIOB, GPIO_Pin_8);
				GPIO_SetBits(GPIOB, GPIO_Pin_9);
				TIM8->CCR3 = speed * 9;
				break;
			}
			default:
				break;
		}
	}
	else
	{
		speed = 0.45 * speed - 45;
		switch(wheelNum)
		{
			case 0:
			{
				GPIO_SetBits(GPIOA, GPIO_Pin_2);
				GPIO_ResetBits(GPIOA, GPIO_Pin_3);
				TIM8->CCR1 = -speed * 9;
				break;
			}
			case 1:
			{
				GPIO_SetBits(GPIOB, GPIO_Pin_0);
				GPIO_ResetBits(GPIOB, GPIO_Pin_1);
				TIM8->CCR2 = -speed * 9;
				break;
			}
			case 2:
			{
				GPIO_SetBits(GPIOB, GPIO_Pin_8);
				GPIO_ResetBits(GPIOB, GPIO_Pin_9);
				TIM8->CCR3 = -speed * 9;
				break;
			}
			default:
				break;
		}
	}
}

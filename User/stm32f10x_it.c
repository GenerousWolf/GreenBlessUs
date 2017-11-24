/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/

#include "stdafx.h"

/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
s16 motorSpeeds[3];
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
#ifndef RTE_CMSIS_RTOS_RTX
void SVC_Handler(void)
{
}
#endif

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
#ifndef RTE_CMSIS_RTOS_RTX
void PendSV_Handler(void)
{
}
#endif

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
#ifndef RTE_CMSIS_RTOS_RTX
void SysTick_Handler(void)
{
	TimingCnt_Decrement();
}
#endif
/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles USART1 interrupt request.
  * @param  None
  * @retval None
  */
void USART1_IRQHandler(void)
{
	/* When the serial recieve a byte, do the work below. */
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
		/* Clear the flag of recieve buffer not empty. */
		USART_ClearITPendingBit(USART1,USART_IT_RXNE);
		/* Get the byte. */
		uint8_t ch = USART_ReceiveData(USART1); 
		/* Do something with this byte. */
		// putchar(ch);
		if(Rx2Counter != MAX_BUFFER_SIZE)
			Rx2Buffer[Rx2Counter++] = ch;
		/* Try not to use printf() or scanf(), for their lacking of efficiency. */
	}
	if(Rx2Counter == 32)
    {
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
		Rx2Counter = 0;
		putchar('R');
		putchar('\n');
    }
	/*if(Rx1Counter == 24)
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
		
	}*/
}

/**
  * @brief  This function handles TIM6 interrupt request.
  * @param  None
  * @retval None
  */ 

void TIM6_IRQHandler(void)
{  
	if (TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET)
    {
		TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
		GetAllFromMPU();			//惯导积分
		courseAngle = AngYaw;
		MotorControl();
		//printf("cA = %f\n", courseAngle);
    }
}

/**
  * @brief  This function handles USART4 interrupt request.
  * @param  None
  * @retval None
  */ 

void USART4_IRQHandler(void)
{
	/* When the serial recieve a byte, do the work below. */
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
		/* Clear the flag of recieve buffer not empty. */
		USART_ClearITPendingBit(USART1,USART_IT_RXNE);
		/* Get the byte. */
		uint8_t ch = USART_ReceiveData(USART1); 
		/* Do something with this byte. */
		if(Rx2Counter != MAX_BUFFER_SIZE)
			Rx2Buffer[Rx2Counter++] = ch;
		/* Try not to use printf() or scanf(), for their lacking of efficiency. */
	}
}

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/

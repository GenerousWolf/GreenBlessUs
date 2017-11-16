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
		if(Rx1Counter != MAX_BUFFER_SIZE)
			Rx1Buffer[Rx1Counter++] = ch;
		/* Try not to use printf() or scanf(), for their lacking of efficiency. */
	}	
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
			
			motorspeedread();			//��ȡ����������
			GetAllFromMPU();			//�ߵ�����
			getSelfAngle();				//��ȡ��ǰ������
			
			Encoder_Reset();			//����������
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
		// putchar(ch);
		if(Rx2Counter != MAX_BUFFER_SIZE)
			Rx2Buffer[Rx2Counter++] = ch;
		/* Try not to use printf() or scanf(), for their lacking of efficiency. */
	}
}

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/

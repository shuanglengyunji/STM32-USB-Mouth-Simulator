/**
  ******************************************************************************
  * @file    stm32_it.c
  * @author  MCD Application Team
  * @version V4.1.0
  * @date    26-May-2017
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and peripherals
  *          interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */


/* Includes ------------------------------------------------------------------*/
#include "stm32_it.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern __IO uint32_t DevRemoteWakeup;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * Function Name  : NMI_Handler
  * Description    : This function handles NMI exception.
  * Input          : None
  * Output         : None
  * Return         : None
  */
void NMI_Handler(void)
{
}

/**
  * Function Name  : HardFault_Handler
  * Description    : This function handles Hard Fault exception.
  * Input          : None
  * Output         : None
  * Return         : None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * Function Name  : MemManage_Handler
  * Description    : This function handles Memory Manage exception.
  * Input          : None
  * Output         : None
  * Return         : None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * Function Name  : BusFault_Handler
  * Description    : This function handles Bus Fault exception.
  * Input          : None
  * Output         : None
  * Return         : None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * Function Name  : UsageFault_Handler
  * Description    : This function handles Usage Fault exception.
  * Input          : None
  * Output         : None
  * Return         : None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * Function Name  : SVC_Handler
  * Description    : This function handles SVCall exception.
  * Input          : None
  * Output         : None
  * Return         : None
  */
void SVC_Handler(void)
{
}

/**
  * Function Name  : DebugMon_Handler
  * Description    : This function handles Debug Monitor exception.
  * Input          : None
  * Output         : None
  * Return         : None
  */
void DebugMon_Handler(void)
{
}

/**
  * Function Name  : PendSV_Handler
  * Description    : This function handles PendSVC exception.
  * Input          : None
  * Output         : None
  * Return         : None
  */
void PendSV_Handler(void)
{
}

/**
  * Function Name  : SysTick_Handler
  * Description    : This function handles SysTick Handler.
  * Input          : None
  * Output         : None
  * Return         : None
  */
void SysTick_Handler(void)
{
}

/******************************************************************************/
/*                 STM32 Peripherals Interrupt Handlers                       */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32xxx.s).                                                */
/******************************************************************************/

/**
  * Function Name  : USB_IRQHandler
  * Description    : This function handles USB Low Priority interrupts
  *                  requests.
  * Input          : None
  * Output         : None
  * Return         : None
  */
void USB_LP_CAN1_RX0_IRQHandler(void)
{
  USB_Istr();
}

/**
  * Function Name  : USBWakeUp_IRQHandler
  * Description    : This function handles USB WakeUp interrupt request.
  * Input          : None
  * Output         : None
  * Return         : None
  */
void USBWakeUp_IRQHandler(void)
{
  EXTI_ClearITPendingBit(EXTI_Line18);
}

/**
  * Function Name  : USART1_IRQHandler
  * Description    : This function handles Usart1 interrupt request.
  * Input          : None
  * Output         : None
  * Return         : None
  */
void USART1_IRQHandler(void)
{
	uint8_t ch;
	
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{ 	
	    //ch = USART1->DR;
		ch = USART_ReceiveData(USART1);
		
		//把接收到的东西发回去
		while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);	/* 等待发送完毕 */
		USART_SendData(USART1, (uint8_t) ch);							/* 发送一个字节数据到USART1 */

	} 
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

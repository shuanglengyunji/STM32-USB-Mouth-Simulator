/**
  ******************************************************************************
  * @file    stm3210b_eval.c
  * @author  MCD Application Team
  * @version V5.0.1
  * @date    05-March-2012
  * @brief   This file provides
  *            - set of firmware functions to manage Leds, push-button and COM ports
  *            - low level initialization functions for SD card (on SPI), SPI serial
  *              flash (sFLASH) and temperature sensor (LM75)
  *          available on STM3210B-EVAL evaluation board from STMicroelectronics.    
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */ 
  
/* Includes ------------------------------------------------------------------*/
#include "stm3210b_eval.h"

/**
  * @brief  Configures LED1 GPIO.
  * @param  None
  * @retval None
  */
void STM_EVAL_LED1_Init(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  
  /* Enable the GPIO_LED Clock */
  RCC_APB2PeriphClockCmd(LED1_GPIO_CLK, ENABLE);
  /* Configure the GPIO_LED pin */
  GPIO_InitStructure.GPIO_Pin = LED1_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(LED1_GPIO_PORT, &GPIO_InitStructure);
}

/**
  * @brief  Configures LED234 GPIO.
  * @param  None
  * @retval None
  */
void STM_EVAL_LED234_Init(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
	
  /* Enable the GPIO_LED Clock */
  RCC_APB2PeriphClockCmd(LED2_GPIO_CLK, ENABLE);
  /* Configure the GPIO_LED pin */
  GPIO_InitStructure.GPIO_Pin = LED2_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(LED2_GPIO_PORT, &GPIO_InitStructure);
	
  /* Enable the GPIO_LED Clock */
  RCC_APB2PeriphClockCmd(LED3_GPIO_CLK, ENABLE);
  /* Configure the GPIO_LED pin */
  GPIO_InitStructure.GPIO_Pin = LED3_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(LED3_GPIO_PORT, &GPIO_InitStructure);
	
  /* Enable the GPIO_LED Clock */
  RCC_APB2PeriphClockCmd(LED4_GPIO_CLK, ENABLE);
  /* Configure the GPIO_LED pin */
  GPIO_InitStructure.GPIO_Pin = LED4_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(LED4_GPIO_PORT, &GPIO_InitStructure);
}

/**
  * @brief  Turns selected LED On.
  * @param  None
  * @retval None
  */
void STM_EVAL_LEDOn(u8 led)
{
	switch(led)
	{
		case LED1:
			LED1_GPIO_PORT->BRR = LED1_PIN;
		break;
		
		case LED2:
			LED2_GPIO_PORT->BRR = LED2_PIN;
		break;
		
		default:
			
		break;
	}
}

/**
  * @brief  Turns selected LED Off.
  * @param  None
  * @retval None
  */
void STM_EVAL_LEDOff(u8 led)
{
	switch(led)
	{
		case LED1:
			LED1_GPIO_PORT->BSRR = LED1_PIN;
		break;
		
		case LED2:
			LED2_GPIO_PORT->BSRR = LED2_PIN;
		break;
		
		default:
			
		break;
	}
}

/**
  * @brief  Toggles the selected LED.
  * @param  None
  * @retval None
  */
void STM_EVAL_LEDToggle(u8 led)
{
	switch(led)
	{
		case LED1:
			LED1_GPIO_PORT->ODR ^= LED1_PIN;
		break;
		
		case LED2:
			LED2_GPIO_PORT->ODR ^= LED2_PIN;
		break;
		
		default:
			
		break;
	}
}

/**
  * @brief  Configures Button GPIO.
  * @param  None
  * @retval None
  */
void STM_EVAL_PBInit(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Enable the BUTTON Clock */
  RCC_APB2PeriphClockCmd(PUSH_BUTTON1_GPIO_CLK | PUSH_BUTTON2_GPIO_CLK | PUSH_BUTTON3_GPIO_CLK | RCC_APB2Periph_AFIO, ENABLE);

  /* Configure Button pin as input floating */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;	//浮空输入
  GPIO_InitStructure.GPIO_Pin = PUSH_BUTTON1_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(PUSH_BUTTON1_GPIO_PORT, &GPIO_InitStructure);
	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;	//浮空输入
  GPIO_InitStructure.GPIO_Pin = PUSH_BUTTON2_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(PUSH_BUTTON2_GPIO_PORT, &GPIO_InitStructure);
	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;	//浮空输入
  GPIO_InitStructure.GPIO_Pin = PUSH_BUTTON3_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(PUSH_BUTTON3_GPIO_PORT, &GPIO_InitStructure);
}

/**
  * @brief  Returns the selected Button state.
  * @param  None. 
  * @retval The Button GPIO pin value.
  */
uint32_t STM_EVAL_PBGetState(u8 pushbutton)
{
	if(pushbutton == PUSH_BUTTON1)
		return GPIO_ReadInputDataBit(PUSH_BUTTON1_GPIO_PORT, PUSH_BUTTON1_PIN);
	else if(pushbutton == PUSH_BUTTON2)
		return GPIO_ReadInputDataBit(PUSH_BUTTON2_GPIO_PORT, PUSH_BUTTON2_PIN);
	else if(pushbutton == PUSH_BUTTON3)
		return GPIO_ReadInputDataBit(PUSH_BUTTON3_GPIO_PORT, PUSH_BUTTON3_PIN);
	else
		return 0;
}

/**
  * @brief  Configures COM port.
  * @param  None
  * @retval None
  */
void STM_EVAL_COM1_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
	/* config USART1 clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA| RCC_APB2Periph_GPIOB, ENABLE);
	
	/* USART1 GPIO config */
	/* Configure USART1 Tx (PA.09) as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	/* Configure USART1 Rx (PA.10) as input floating */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	/* USART1 mode config */
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);
	
	NVIC_InitTypeDef NVIC_InitStructure;
	
	/* Enable the USARTy Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;	 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;	//主优先级开得比较高，从优先级比较低
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	/* 使能串口1接收中断 */
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	
	USART_Cmd(USART1, ENABLE);
}
    
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

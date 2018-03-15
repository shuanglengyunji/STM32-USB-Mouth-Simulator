/**
  ******************************************************************************
  * @file    hw_config.c
  * @author  MCD Application Team
  * @version V4.1.0
  * @date    26-May-2017
  * @brief   Hardware Configuration & Setup
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
#include "hw_config.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
__IO uint8_t PrevXferComplete = 1;
ErrorStatus HSEStartUpStatus;
EXTI_InitTypeDef EXTI_InitStructure;

/* Extern variables ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void IntToUnicode (uint32_t value , uint8_t *pbuf , uint8_t len);

/* Private functions ---------------------------------------------------------*/

/**
  * Function Name  : Set_System
  * Description    : Configures Main system clocks & power.
  * Input          : None.
  * Return         : None.
  */
void Set_System(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
	
  /*   
	   At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_stm32xxx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32xxx.c file
  */ 
  
  /******************************************/
  /*  1.Enable the PWR clock         		*/
  /******************************************/
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  
  /********************************************/
  /*  2.Configure USB DM/DP pins                */
  /********************************************/
  
  /* Configure USB DM/DP pin. This is optional, and maintained only for user guidance.
  For the STM32L products there is no need to configure the PA12/PA11 pins couple 
  as Alternate Function */

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  /* Enable all GPIOs Clock*/
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ALLGPIO, ENABLE);

  /********************************************/
  /*  3.Enable the USB PULL UP                */
  /********************************************/

#ifdef USB_LOW_PWR_MGMT_SUPPORT
  
  /**********************************************************************/
  /*  4.Configure the EXTI line 18 connected internally to the USB IP   */
  /**********************************************************************/
  
  EXTI_ClearITPendingBit(EXTI_Line18);
  
  EXTI_InitStructure.EXTI_Line = EXTI_Line18;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

#endif  /* USB_LOW_PWR_MGMT_SUPPORT */
  
} 
 
/**
  * Function Name  : Set_USBClock
  * Description    : Configures USB Clock input (48MHz).
  * Input          : None.
  * Output         : None.
  * Return         : None.
  */
void Set_USBClock(void)
{
  /* Select USBCLK source */
  RCC_USBCLKConfig(RCC_USBCLKSource_PLLCLK_1Div5);
  
  /* Enable the USB clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB, ENABLE);
}

/**
  * Function Name  : Leave_LowPowerMode.
  * Description    : Restores system clocks and power while exiting suspend mode.
  * Input          : None.
  * Output         : None.
  * Return         : None.
  */
void Leave_LowPowerMode(void)
{
  DEVICE_INFO *pInfo = &Device_Info;
  
  /* Set the device state to the correct state */
  if (pInfo->Current_Configuration != 0)
  {
    /* Device configured */
    bDeviceState = CONFIGURED;
  }
  else
  {
    bDeviceState = ATTACHED;
  }
  
#ifdef USB_LOW_PWR_MGMT_SUPPORT
  /* Reset SLEEPDEEP bit of Cortex System Control Register */
  SCB->SCR &= (uint32_t)~((uint32_t)(SCB_SCR_SLEEPDEEP_Msk | SCB_SCR_SLEEPONEXIT_Msk));
  
  /*Enable SystemCoreClock*/
  SystemInit();
    
#endif /* USB_LOW_PWR_MGMT_SUPPORT */
}

/**
  * Function Name  : USB_Interrupts_Config.
  * Description    : Configures the USB interrupts.
  * Input          : None.
  * Output         : None.
  * Return         : None.
  */
void USB_Interrupts_Config(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  /* 2 bit for pre-emption priority, 2 bits for subpriority */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  
  /* Enable the USB interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;		//这个优先级配的比较低，可能是希望USB在后台工作不要打断其他中断
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
#ifdef USB_LOW_PWR_MGMT_SUPPORT
  /* Enable the USB Wake-up interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USBWakeUp_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_Init(&NVIC_InitStructure); 
#endif /* USB_LOW_PWR_MGMT_SUPPORT */ 

}

/**
  * Function Name  : USB_Cable_Config.
  * Description    : Software Connection/Disconnection of USB Cable.
  * Input          : NewState: new state.
  * Output         : None.
  * Return         : None
  */
void USB_Cable_Config (FunctionalState NewState)
{
  if (NewState != DISABLE)
  {

  }
  else
  {

  }
}

/**
  * Function Name : Usb_Mouse_Send.
  * Description   : prepares buffer to be sent containing mouse event infos.
  * Input         : left	left key
					right	right key
					middle	middle key
					x		x move
					y		y move
					z		z move
  * Output        : None.
  * Return value  : None.
  */
void Usb_Mouse_Send(u8 byte1, u8 byte2, u8 byte3, u8 byte4)
{
	uint8_t Mouse_Buffer[4] = {0, 0, 0, 0};
	
	while(!PrevXferComplete){}	//等待上一帧的内容发完
	
	/* prepare buffer to send */
	Mouse_Buffer[0] = byte1;
	Mouse_Buffer[1] = byte2;
	Mouse_Buffer[2] = byte3;
	Mouse_Buffer[3] = byte4;

	/* Reset the control token to inform upper layer that a transfer is ongoing */
	PrevXferComplete = 0;

	/* Copy mouse position info in ENDP1 Tx Packet Memory Area*/
	USB_SIL_Write(EP1_IN, Mouse_Buffer, 4);

	/* Enable endpoint for transmission */
	SetEPTxValid(ENDP1);
}

/**
  * Function Name : Usart_Mouse_Send.
  * Description   : prepares buffer to be sent containing mouse event infos.
  * Input         : left	left key
					right	right key
					middle	middle key
					x		x move
					y		y move
					z		z move
  * Output        : None.
  * Return value  : None.
  */
void Usart_Mouse_Send(u8 byte1, u8 byte2, u8 byte3, u8 byte4)
{
	//帧头是 0xAB 0xCD
	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);	/* 等待发送完毕 */
	USART_SendData(USART1, 0xAB);
	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);	/* 等待发送完毕 */
	USART_SendData(USART1, 0xCD);
	
	//发送4个字节
	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);	/* 等待发送完毕 */
	USART_SendData(USART1, byte1);
	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);	/* 等待发送完毕 */
	USART_SendData(USART1, byte2);
	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);	/* 等待发送完毕 */
	USART_SendData(USART1, byte3);
	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);	/* 等待发送完毕 */
	USART_SendData(USART1, byte4);
}

/**
  * Function Name  : Get_SerialNum.
  * Description    : Create the serial number string descriptor.
  * Input          : None.
  * Output         : None.
  * Return         : None.
  */
void Get_SerialNum(void)
{
  uint32_t Device_Serial0, Device_Serial1, Device_Serial2;

  Device_Serial0 = *(uint32_t*)ID1;
  Device_Serial1 = *(uint32_t*)ID2;
  Device_Serial2 = *(uint32_t*)ID3;
  
  Device_Serial0 += Device_Serial2;

  if (Device_Serial0 != 0)
  {
    IntToUnicode (Device_Serial0, &Joystick_StringSerial[2] , 8);
    IntToUnicode (Device_Serial1, &Joystick_StringSerial[18], 4);
  }
}

/**
  * Function Name  : HexToChar.
  * Description    : Convert Hex 32Bits value into char.
  * Input          : None.
  * Output         : None.
  * Return         : None.
  */
static void IntToUnicode (uint32_t value , uint8_t *pbuf , uint8_t len)
{
  uint8_t idx = 0;
  
  for( idx = 0 ; idx < len ; idx ++)
  {
    if( ((value >> 28)) < 0xA )
    {
      pbuf[ 2* idx] = (value >> 28) + '0';
    }
    else
    {
      pbuf[2* idx] = (value >> 28) + 'A' - 10; 
    }
    
    value = value << 4;
    
    pbuf[ 2* idx + 1] = 0;
  }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

///**
//  * Function Name : Joystick_Send.
//  * Description   : prepares buffer to be sent containing Joystick event infos.
//  * Input         : Keys: keys received from terminal.
//  * Output        : None.
//  * Return value  : None.
//  */
//void Joystick_Send(uint8_t Keys)
//{
//  uint8_t Mouse_Buffer[4] = {0, 0, 0, 0};
//  int8_t X = 0, Y = 0;
//  
//  switch (Keys)
//  {
//    case JOY_LEFT:
//      X -= CURSOR_STEP;
//      break;
//    case JOY_RIGHT:
//      X += CURSOR_STEP;
//      break;
//    case JOY_UP:
//      Y -= CURSOR_STEP;
//      break;
//    case JOY_DOWN:
//      Y += CURSOR_STEP;
//      break;
//    default:
//      return;
//  }
//  /* prepare buffer to send */
//  Mouse_Buffer[1] = X;
//  Mouse_Buffer[2] = Y;
//  
//  /* Reset the control token to inform upper layer that a transfer is ongoing */
//  PrevXferComplete = 0;
//  
//  /* Copy mouse position info in ENDP1 Tx Packet Memory Area*/
//  USB_SIL_Write(EP1_IN, Mouse_Buffer, 4);
//  
//  /* Enable endpoint for transmission */
//  SetEPTxValid(ENDP1);

//}


///**
//  * Function Name  : Joy_Emul.
//  * Description    : Gets Pointer Data
//					Emul = Emulate, means a simulator. This function draw a square with the mouse it controled. 
//  * Input          : None.
//  * Output         : None.
//  * Return         : None.
//  */
//void Joy_Emul(void)
//{
//  uint8_t Mouse_Buffer[4] = {0, 0, 0, 0};
//  uint8_t X = 0, Y = 0; 
//  static uint8_t Sens = 0;
//  static uint8_t Step = 0;
//  
//  Delay(0x0FFFF);	//这个函数默认自己会被不间断循环调用，所以需要延时一段时间防止鼠标运动过快
//  
//  if (Step == 35)
//  {
//    Step = 0;
//    Sens++;
//  }
//  
//  if(Sens == 0)
//  {
//    X = Step++;
//    Y = 0;
//  }
//  
//  if(Sens == 1)
//  {
//    Y = Step++;
//    X = 0;
//  }      
//  if (Sens==2)
//  {
//    X = 256 - Step++;
//    Y = 0;
//  } 
//  
//  if (Sens == 3)
//  {
//    Y = 256 - Step++;
//    X = 0;
//  }
//  
//  if (Sens == 4)
//  { 
//    Sens = 0;
//    X = 0;
//    Y = 0;
//  }
//  
//  Mouse_Buffer[0] = 0;
//  Mouse_Buffer[1] = X;
//  Mouse_Buffer[2] = Y;
//  Mouse_Buffer[3] = 0;
//  
//  /* Reset the control token to inform upper layer that a transfer is ongoing */
//  PrevXferComplete = 0;
//  /* Copy mouse position info in ENDP1 Tx Packet Memory Area*/
//  USB_SIL_Write(EP1_IN, Mouse_Buffer, 4);
//  /* Enable endpoint for transmission */
//  SetEPTxValid(ENDP1);
//}


///**
//  * Function Name : JoyState.
//  * Description   : Decodes the Joystick direction.
//  * Input         : None.
//  * Output        : None.
//  * Return value  : The direction value.
//  */
//uint8_t JoyState(void)
//{
////	/* "right" key is pressed */
////	if (STM_EVAL_PBGetState(Button_RIGHT))
////	{
////		return JOY_RIGHT;
////	}
////	
////	/* "left" key is pressed */
////	if (STM_EVAL_PBGetState(Button_LEFT))
////	{
////	return JOY_LEFT;
////	}
////	
////	/* "up" key is pressed */
////	if (STM_EVAL_PBGetState(Button_UP))
////	{
////		return JOY_UP;
////	}
////	
////	/* "down" key is pressed */
////	if (STM_EVAL_PBGetState(Button_DOWN))
////	{
////		return JOY_DOWN;
////	}
////	
////	/* No key is pressed */
////	else
////	{
////		return 0;
////	} 

//	return 0;
//}


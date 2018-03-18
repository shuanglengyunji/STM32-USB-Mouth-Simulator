/**
  ******************************************************************************
  * @file    main.c
  * @author  Liu Han
  * @brief   Joystick Mouse
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "hw_config.h"
#include <stdio.h>
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
u32 Systick_5ms = 0;	//KEY
u32 Systick_50ms = 0;	//LED
u8 Led_flicker_Mode = 0;	//LED的闪烁模式
u8 com_buff[4] = {0, 0, 0, 0};
u8 com_flag = 0;

/* Extern variables ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
static void SysTick_Init(void);

void Check_LED(void)
{
	static u8 counter_led = 0;
	
	switch(Led_flicker_Mode)
	{
		case 0:
			STM_EVAL_LEDOff(LED1);	//关闭LED
		break;
		
		case 1:
			counter_led++;
			if( counter_led % 1 == 0 )	//50ms变换一下
			{
				STM_EVAL_LEDToggle(LED1);
			}
			if(counter_led >= 2)		//只持续闪0.1s
			{
				counter_led = 0;
				Led_flicker_Mode = 0;	//闪完之后就回到不闪的模式去
			}
		break;
		
		default:
			STM_EVAL_LEDOn(LED1);	//出现了不应该出现的情况，LED常亮
		break;
	}
}

/**
  * Function Name  : main
  * Description    : Main
  */
int main(void)
{
	/*  Init USB Driver. */
	Set_System();	//Enable the PWR clock
					//Configure USB DM/DP pins
					//Enable the USB PULL UP
					//Configure the EXTI line 18 connected internally to the USB IP (If defined USB_LOW_PWR_MGMT_SUPPORT)
	USB_Interrupts_Config();	//2 bit for pre-emption priority, 2 bits for subpriority
								//Enable the USB interrupt
								//Enable the Key EXTI line Interrupt
	Set_USBClock();	//Select USB clock source and Init it
	USB_Init();	//Init USB peripheral
	
	/* Init Systick */
	SysTick_Init();	//Init and enable Systick.
	
	/* Init other peripheral */
	STM_EVAL_USART1_Init();	//Init Usart1 in EXIT mode
	STM_EVAL_LED1_Init();	//LED1 Port Init
	printf("串口1初始化完毕\r\n");
	
	/* Init communitcation port with master */
	STM_EVAL_USART3_Init();	//Init Usart3 in EXIT mode
	printf("串口3初始化完毕\r\n");
	
	while (1)
	{
		//KEY
		if(Systick_5ms >= 5)
		{
			Systick_5ms = 0;
			
		}
		
		//LED
		if(Systick_50ms >= 50)
		{
			Systick_50ms = 0;
			
			Check_LED();
		}
		
		//USB工作正常 且 串口接收到数据
		if(bDeviceState == CONFIGURED && com_flag == 1)
		{
			//发送USB数据
			Usb_Mouse_Send(com_buff[0],com_buff[1],com_buff[2],com_buff[3]);
			Led_flicker_Mode = 1;												//指示灯闪烁
			
			//printf("0x%02x 0x%02x 0x%02x 0x%02x\r\n",(uint16_t)com_buff[0],(uint16_t)com_buff[1],(uint16_t)com_buff[2],(uint16_t)com_buff[3]);
			
			//清发送flag
			com_flag = 0;
		}
	}
}

/**
  * Function Name  : USART3_IRQHandler
  * Description    : This function handles Usart1 interrupt request.
  * Input          : None
  * Output         : None
  * Return         : None
  */
void USART3_IRQHandler(void)
{
	uint8_t ch;
	
	static u8 step = 0;
	
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
	{ 	
	    //ch = USART3->DR;
		ch = USART_ReceiveData(USART3);
		
		//如果上一帧没有送入USB口的BUFF，则不接收下一帧
		if(com_flag)
		{
			return;
		}
		
		switch(step)
		{
			case 0:
				if(ch == 0xAB)	//帧头1
					step = 1;
				else
					step = 0;
			break;
			
			case 1:
				if(ch == 0xCD)	//帧头2
					step = 2;
				else
					step = 0;
			break;
				
			case 2:
				com_buff[0] = ch;	//byte1
				step = 3;
			break;
			
			case 3:
				com_buff[1] = ch;	//byte2
				step = 4;
			break;
						
			case 4:
				com_buff[2] = ch;	//byte3
				step = 5;
			break;
			
			case 5:
				com_buff[3] = ch;	//byte4
				com_flag = 1;	//flag
				step = 0;
			break;
			
			default:
				step = 0;
			break;
		}
	}
}

/**
  * Function Name  : USART1_IRQHandler
  * Description    : This function handles Usart1 interrupt request.
					 对电脑的调试端口
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
		
//		//把接收到的东西发回去
//		while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);	/* 等待发送完毕 */
//		USART_SendData(USART1, (uint8_t) ch);							/* 发送一个字节数据到USART1 */
		
	}
}

/**
  * @brief  启动系统滴答定时器 SysTick
  * @param  无
  * @retval 无
  */
/*******************************************************************************
* Function Name  : SysTick_Init
* Description    : Config and start SysTick
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
static void SysTick_Init(void)
{
	/* SystemCoreClock / 1000    1ms中断一次
	 * SystemCoreClock / 100000	 10us中断一次
	 * SystemCoreClock / 1000000 1us中断一次
	 */

	if (SysTick_Config(SystemCoreClock / 1000))	// ST3.5.0库版本
	{ 
		/* Capture error */ 
		while (1);
	}
	
//	SysTick->CTRL &= ~ SysTick_CTRL_ENABLE_Msk;	// 关闭滴答定时器
	SysTick->CTRL |=  SysTick_CTRL_ENABLE_Msk;	// 使能滴答定时器
}

#ifdef  USE_FULL_ASSERT
/*******************************************************************************
* Function Name  : assert_failed
* Description    : Reports the name of the source file and the source line number
*                  where the assert_param error has occurred.
* Input          : - file: pointer to the source file name
*                  - line: assert_param error line source number
* Output         : None
* Return         : None
*******************************************************************************/
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {}
}
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/


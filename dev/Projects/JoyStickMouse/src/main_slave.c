/**
  ******************************************************************************
  * @file    main.c
  * @author  Liu Han
  * @brief   Joystick Mouse
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "hw_config.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
u32 Systick_5ms = 0;	//KEY
u32 Systick_100ms = 0;	//LED
u8 Led_flicker_Mode = 0;	//LED的闪烁模式
u8 com_receive[4] = {0, 0, 0, 0};
u8 com_receive_flag = 0;

/* Extern variables ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
static void SysTick_Init(void);

/**
  * @brief  Inserts a delay time.
  * @param  nTime: specifies the delay time length, in milliseconds.
  * @retval None
  */
void delay_ms(u16 time)
{    
	u16 i=0;  
	while(time--)
	{
		i=12000;  //自己定义
		while(i--) ;    
	}
}

/*******************************************************************************
* Function Name  : main.
* Description    : main routine.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
int main(void)
{
	/*
		1.Enable the PWR clock
		2.Configure USB DM/DP pins
		3.Enable the USB PULL UP
		4.Configure the Joystick buttons in GPIO mode
		5.Configure the EXTI line 18 connected internally to the USB IP (If defined USB_LOW_PWR_MGMT_SUPPORT)
		6.Init Usart1 in EXIT mode
	*/
	Set_System();
  
	/*
		2 bit for pre-emption priority, 2 bits for subpriority
		Enable the USB interrupt
		Enable the USB Wake-up interrupt(Internal interrupr)
		Enable the Key EXTI line Interrupt
	*/
	USB_Interrupts_Config();
	
	/*
		Select USB clock source and Init it.
	*/
	Set_USBClock();
	
	/*
		Init USB peripheral and Begin to communicate with computer.
		The usb peripheral was enabled at this time.
	*/
	USB_Init();
	
	/*
		Init and enable Systick.
	*/
	SysTick_Init();
	
	while (1)
	{
		//KEY
		if(Systick_5ms >= 5)
		{
			Systick_5ms = 0;
			
		}
		
		//LED
		if(Systick_100ms >= 100)
		{
			Systick_100ms = 0;
			
			static u8 counter_led = 0;
			
			switch(Led_flicker_Mode)
			{
				case 0:
					STM_EVAL_LEDOff(LED1);	//关闭LED
				break;
				
				case 1:
					counter_led++;
					if( counter_led % 1 == 0 )	//200ms变换一下
					{
						STM_EVAL_LEDToggle(LED1);
					}
					if(counter_led >= 10)		//只持续闪1s
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
		
		//USB工作正常 且 串口接收到数据
		if(bDeviceState == CONFIGURED && com_receive_flag == 1)
		{
			com_receive_flag = 0;
			Usb_Mouse_Send(com_receive[0],com_receive[1],com_receive[2],com_receive[3]);
		}
	}
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
	
	static u8 step = 0;
	
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{ 	
	    //ch = USART1->DR;
		ch = USART_ReceiveData(USART1);
		
//		//把接收到的东西发回去
//		while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);	/* 等待发送完毕 */
//		USART_SendData(USART1, (uint8_t) ch);							/* 发送一个字节数据到USART1 */
		
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
				
			case 2:
				com_receive[0] = ch;	//byte1
				step = 3;
			break;
			
			case 3:
				com_receive[1] = ch;	//byte2
				step = 4;
			break;
						
			case 4:
				com_receive[2] = ch;	//byte3
				step = 5;
			break;
			
			case 5:
				com_receive[3] = ch;	//byte4
				com_receive_flag = 1;	//flag
				step = 0;
			break;
			
			default:
				step = 0;
			break;
		}
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


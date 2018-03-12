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

/**
  * @brief  Mouse_Cmd.
  * @param  left.right.middle.
			x.y.z.
  * @retval None
  */
static void Mouse_Cmd(u8 left, u8 right, u8 middle, int8_t x, int8_t y, int8_t z)
{
	//byte1
	uint8_t tmp_byte1 = 0x00;
	if(left)	tmp_byte1 |= 0x01;
	if(right)	tmp_byte1 |= 0x02;
	if(middle)	tmp_byte1 |= 0x04;
	//byte2
	uint8_t tmp_byte2 = x;
	//byte3
	uint8_t tmp_byte3 = y;
	//byte4
	uint8_t tmp_byte4 = z;
	
	//发送
	Usb_Mouse_Send(tmp_byte1,tmp_byte2,tmp_byte3,tmp_byte4);
	Usart_Mouse_Send(tmp_byte1,tmp_byte2,tmp_byte3,tmp_byte4);
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
			
			//按键检测 + 延时防抖
			
			///////////////////////////////////////////////////////////////////
			//KEY1
			
			u32 key1 = 0;					//本次按键状态
			static u32 key_last1 = 0;		//上一次的按键状态
			
			key1 = STM_EVAL_PBGetState(PUSH_BUTTON1);	//取当前按键状态
			if(key1)
			{
				key_last1++;
				if(key_last1 >= 4)		//防抖时间间隔  4*5ms = 20ms
				{
					//满足要求，点击一次
					
				}
			}
			else
			{
				key_last1 = 0;		//对上一次按键状态清零
			}
			
			///////////////////////////////////////////////////////////////////
			//KEY2
			
			u32 key2 = 0;					//本次按键状态
			static u32 key_last2 = 0;		//上一次的按键状态
			
			key2 = STM_EVAL_PBGetState(PUSH_BUTTON2);	//取当前按键状态
			if(key2)
			{
				key_last2++;
				if(key_last2 >= 4)		//防抖时间间隔  4*5ms = 20ms
				{
					//满足要求，点击一次
					
				}
			}
			else
			{
				key_last2 = 0;		//对上一次按键状态清零
			}
			
			///////////////////////////////////////////////////////////////////
			//KEY3
			
			u32 key3 = 0;					//本次按键状态
			static u32 key_last3 = 0;		//上一次的按键状态
			
			key3 = STM_EVAL_PBGetState(PUSH_BUTTON3);	//取当前按键状态
			if(key3)
			{
				key_last3++;
				if(key_last3 >= 4)		//防抖时间间隔  4*5ms = 20ms
				{
					//满足要求，点击一次
					
				}
			}
			else
			{
				key_last3 = 0;		//对上一次按键状态清零
			}
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
		
		//如果USB正常，才进行：1.PS2捕捉  2.USB发送
		if(bDeviceState == CONFIGURED)
		{
			Mouse_Cmd(0,0,0,0,0,0);
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


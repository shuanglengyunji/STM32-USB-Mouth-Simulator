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
__IO uint8_t PrevXferComplete = 1;
u32 Systick_5ms = 0;	//KEY
u32 Systick_100ms = 0;	//LED
u8 Usb_Left_Key_Flag = 0;		//USB��Ҫ�������
u8 Usb_Right_Key_Flag = 0;		//USB��Ҫ�����Ҽ�
u8 Usart_Left_Key_Flag = 0;		//Usart��Ҫ�������
u8 Usart_Right_Key_Flag = 0;	//Usart��Ҫ�����Ҽ�
/* Extern variables ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
static void SysTick_Init(void);

/**
  * @brief  Inserts a delay time.
  * @param  nTime: specifies the delay time length, in milliseconds.
  * @retval None
  */
static void delay_ms(u16 time)
{    
	u16 i=0;  
	while(time--)
	{
		i=12000;  //�Լ�����
		while(i--) ;    
	}
}

/**
  * @brief  Usart Send Left key CMD.
  * @param  None
  * @retval None
  */
static void Usart_Left_Key(void)
{
	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);	/* �ȴ�������� */
	USART_SendData(USART1, 0x01);									/* �������ָ�� */
}

/**
  * @brief  Usart Send Right key CMD.
  * @param  None
  * @retval None
  */
static void Usart_Right_Key(void)
{
	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);	/* �ȴ�������� */
	USART_SendData(USART1, 0x02);									/* �����Ҽ�ָ�� */
}

/**
  * @brief  Usb Send Left key CMD.
  * @param  None
  * @retval None
  */
static void Usb_Left_Key(void)
{
	while(!PrevXferComplete){}
	Leftkey_Send(ENABLE);
	while(!PrevXferComplete){}
	Leftkey_Send(DISABLE);
}

/**
  * @brief  Usb Send Right key CMD.
  * @param  None
  * @retval None
  */
static void Usb_Right_Key(void)
{
	while(!PrevXferComplete){}
	Rightkey_Send(ENABLE);
	while(!PrevXferComplete){}
	Rightkey_Send(DISABLE);	
}

/**
  * @brief  Move.
  * @param  u8 Direction.
  * @retval None
  */
static void Usb_Move(u8 Direction)
{
	while(!PrevXferComplete){}
	switch(Direction)
	{
		case UP:
			Joystick_Send(JOY_UP);
		break;
		
		case DOWN:
			Joystick_Send(JOY_DOWN);
		break;
		
		case LEFT:
			Joystick_Send(JOY_LEFT);
		break;
		
		case RIGHT:
			Joystick_Send(JOY_RIGHT);
		break;
		
		default:
			return;
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
			
			//������� + ��ʱ����
			
			u32 key = 0;					//���ΰ���״̬
			static u32 key_last = 0;		//��һ�εİ���״̬
			
			key = STM_EVAL_PBGetState();	//ȡ��ǰ����״̬
			if(key)
			{
				key_last++;
				if(key_last >= 4)		//����ʱ����  4*5ms = 20ms
				{
					//����Ҫ�󣬵��һ��
					
					//���
					Usb_Left_Key_Flag = 1;	
					Usart_Left_Key_Flag = 1;
					
					//�Ҽ�
//					Usb_Right_Key_Flag = 1;
//					Usart_Right_Key_Flag = 1;
				}
			}
			else
			{
				key_last = 0;		//����һ�ΰ���״̬����
			}
			
		}
		
		//LED
		if(Systick_100ms >= 100)
		{
			Systick_100ms = 0;
			
			static u8 counter_led = 0;
			
			counter_led++;
			if(counter_led >= 5)
			{
				counter_led = 0;
				STM_EVAL_LEDToggle(LED1);
			}
			
		}
		
		//USB��������һֱ�ܣ�Ҫ��Ҫ���͵����Ϸ�
		if(bDeviceState == CONFIGURED)
		{
			if(Usb_Left_Key_Flag)
			{
				Usb_Left_Key_Flag = 0;
				
				Usb_Left_Key();
			}
			
			if(Usb_Right_Key_Flag)
			{
				Usb_Right_Key_Flag = 0;
				
				Usb_Right_Key();
			}
		}
		
		//Usart��������һֱ�ܣ���ʱ����
		if(Usart_Left_Key_Flag)
		{
			Usart_Left_Key_Flag = 0;
			
			Usart_Left_Key();
		}
		if(Usart_Right_Key_Flag)
		{
			Usart_Right_Key_Flag = 0;
			
			Usart_Right_Key();
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
		
//		//�ѽ��յ��Ķ�������ȥ
//		while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);	/* �ȴ�������� */
//		USART_SendData(USART1, (uint8_t) ch);							/* ����һ���ֽ����ݵ�USART1 */
		
		if(ch == 0x01)
		{
			Usb_Left_Key_Flag = 1;
		}
		else if(ch == 0x02)
		{
			Usb_Right_Key_Flag = 1;
		}
	} 
}

/**
  * @brief  ����ϵͳ�δ�ʱ�� SysTick
  * @param  ��
  * @retval ��
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
	/* SystemCoreClock / 1000    1ms�ж�һ��
	 * SystemCoreClock / 100000	 10us�ж�һ��
	 * SystemCoreClock / 1000000 1us�ж�һ��
	 */

	if (SysTick_Config(SystemCoreClock / 1000))	// ST3.5.0��汾
	{ 
		/* Capture error */ 
		while (1);
	}
	
//	SysTick->CTRL &= ~ SysTick_CTRL_ENABLE_Msk;	// �رյδ�ʱ��  
	SysTick->CTRL |=  SysTick_CTRL_ENABLE_Msk;	// ʹ�ܵδ�ʱ��
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


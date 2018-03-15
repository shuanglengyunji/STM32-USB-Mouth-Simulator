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
u32 Systick_5ms = 0;		//KEY
u32 Systick_100ms = 0;		//LED
u8 Led_flicker_Mode = 0;	//LED����˸ģʽ

u8 send_buff[4] = {0, 0, 0, 0};
u8 send_flag = 0;

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
		i=12000;  //�Լ�����
		while(i--) ;    
	}
}

///**
//  * @brief  Mouse_Cmd.
//  * @param  left.right.middle.
//			x.y.z.
//  * @retval None
//  */
//static void Mouse_To_Byte(u8 left, u8 right, u8 middle, int8_t x, int8_t y, int8_t z)
//{
//	//byte1
//	uint8_t tmp_byte1 = 0x00;
//	if(left)	tmp_byte1 |= 0x01;
//	if(right)	tmp_byte1 |= 0x02;
//	if(middle)	tmp_byte1 |= 0x04;
//	//byte2
//	uint8_t tmp_byte2 = x;
//	//byte3
//	uint8_t tmp_byte3 = y;
//	//byte4
//	uint8_t tmp_byte4 = z;
//}

/*******************************************************************************
* Function Name  : main.
* Description    : main routine.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
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
	STM_EVAL_COM1_Init();	//Init Usart1 in EXIT mode
	STM_EVAL_LED1_Init();	//LED1 Port Init
	
	STM_EVAL_LED234_Init();	//LED234 Port Init
	STM_EVAL_PBInit();		//PUSH BUTTON Port Init
	
	
	while (1)
	{
		//KEY
		if(Systick_5ms >= 5)
		{
			Systick_5ms = 0;
			
			//������� + ��ʱ����
			
			///////////////////////////////////////////////////////////////////
			//KEY1
			
			u32 key1 = 0;					//���ΰ���״̬
			static u32 key_last1 = 0;		//��һ�εİ���״̬
			
			key1 = STM_EVAL_PBGetState(PUSH_BUTTON1);	//ȡ��ǰ����״̬
			if(key1)
			{
				key_last1++;
				if(key_last1 >= 4)		//����ʱ����  4*5ms = 20ms
				{
					//����Ҫ�󣬵��һ��
					
				}
			}
			else
			{
				key_last1 = 0;		//����һ�ΰ���״̬����
			}
			
			///////////////////////////////////////////////////////////////////
			//KEY2
			
			u32 key2 = 0;					//���ΰ���״̬
			static u32 key_last2 = 0;		//��һ�εİ���״̬
			
			key2 = STM_EVAL_PBGetState(PUSH_BUTTON2);	//ȡ��ǰ����״̬
			if(key2)
			{
				key_last2++;
				if(key_last2 >= 4)		//����ʱ����  4*5ms = 20ms
				{
					//����Ҫ�󣬵��һ��
					
				}
			}
			else
			{
				key_last2 = 0;		//����һ�ΰ���״̬����
			}
			
			///////////////////////////////////////////////////////////////////
			//KEY3
			
			u32 key3 = 0;					//���ΰ���״̬
			static u32 key_last3 = 0;		//��һ�εİ���״̬
			
			key3 = STM_EVAL_PBGetState(PUSH_BUTTON3);	//ȡ��ǰ����״̬
			if(key3)
			{
				key_last3++;
				if(key_last3 >= 4)		//����ʱ����  4*5ms = 20ms
				{
					//����Ҫ�󣬵��һ��
					
				}
			}
			else
			{
				key_last3 = 0;		//����һ�ΰ���״̬����
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
					STM_EVAL_LEDOff(LED1);	//�ر�LED
				break;
				
				case 1:
					counter_led++;
					if( counter_led % 1 == 0 )	//200ms�任һ��
					{
						STM_EVAL_LEDToggle(LED1);
					}
					if(counter_led >= 10)		//ֻ������1s
					{
						counter_led = 0;
						Led_flicker_Mode = 0;	//����֮��ͻص�������ģʽȥ
					}
				break;
				
				default:
					STM_EVAL_LEDOn(LED1);	//�����˲�Ӧ�ó��ֵ������LED����
				break;
			}
		}
		
		//USB�������� �� PS/2���յ�����
		if(bDeviceState == CONFIGURED && send_flag == 1)
		{
			Usart_Mouse_Send(send_buff[0],send_buff[1],send_buff[2],send_buff[3]);				//���ڷ��ͳ�ȥ
			Usb_Mouse_Send(send_buff[0],send_buff[1],send_buff[2],send_buff[3]);				//USB���ͳ�ȥ
			send_flag = 0;
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


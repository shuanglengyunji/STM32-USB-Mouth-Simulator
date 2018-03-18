/**
  ******************************************************************************
  * @file    main.c
  * @author  Liu Han
  * @brief   Joystick Mouse
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "hw_config.h"
#include "interface.h"     //�ײ�ӿں���
#include "HOST_SYS.H"      //������������
#include <stdio.h>
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
u32 Systick_5ms = 0;		//KEY
u32 Systick_50ms = 0;		//LED
u8 Led_flicker_Mode = 0;	//LED����˸ģʽ

u8 send_buff[4] = {0, 0, 0, 0};
u8 send_flag = 0;

uint8_t UserBuffer[256];

/* Extern variables ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
static void SysTick_Init(void);

/**
  * Function Name  : Check_CH375
  * Description    : The function of ch375
  */
void Check_CH375(void)
{
	uint8_t res,i,j;
	uint16_t l;
	
	if( CH375CheckConnect() == USBD_CONNECT )          /* �ռ�⵽һ���豸���룬��Ҫö�� */
	{
		printf ( "�豸���ӳɹ�\r\n" );
		
		//��ʼö�ٲ���
		
		printf ( "��ʼö��\r\n" );
		
		/* ���߸�λ */
		res = CH375BusReset();                   
		if( res != USB_INT_SUCCESS ) 
		{
			printf("Bus Reset Erro\n");
		}
		else
		{
			printf ( "���߸�λ�ɹ�\r\n" );
		}
		
		delay_ms( 50 );                          /* �ȴ��豸�ȶ� */

		/* ��ȡ�豸������ */			
		res = CH375GetDeviceDesc( UserBuffer,&l); 
		if( res == USB_INT_SUCCESS )
		{
			printf ( "��ȡ�豸�������ɹ�\r\n" );
			for( i = 0; i < l; i++ )
			{
				printf("0x%02x ",(uint16_t)UserBuffer[i]);
			}
			printf ("\r\n");
		}
		else 
		{
			printf ( "��ȡ�豸������ʧ��\r\n" );
			printf("Get Device Descr Erro:0x%02x\n",(uint16_t)res );
		}
		
		/* ���õ�ַ */
		res = CH375SetDeviceAddr( 2 );
		if( res!= USB_INT_SUCCESS )
		{
			printf ( "���õ�ַʧ��\r\n" );
			printf ("Set Addr Erro:0x%02x\n",(uint16_t)res );	
		}
		else
		{
			printf ( "���õ�ַ�ɹ�\r\n" );
		}
		
		/* ��ȡ���������� */
		res = CH375GetConfDesc( UserBuffer,&l); 
		if( res== USB_INT_SUCCESS )
		{
			printf ( "��ȡ�����������ɹ�\r\n" );
			for( i = 0; i < l; i++ )
			{
				printf("0x%02x ",(uint16_t)UserBuffer[i]);
			}
			printf ("\r\n");					
		}
		else 
		{
			printf ( "��ȡ����������ʧ��\r\n" );
			printf ("Get Conf Descr Erro:0x%02x\n",(uint16_t)res );	
		}			
		
		/* �������� */
		res = CH375SetDeviceConf( 1 );
		if( res != USB_INT_SUCCESS ) 
		{
			printf ( "��������ʧ��\r\n" );
			printf("Set Config Erro\n");
		}
		else
		{
			printf ( "�������óɹ�\r\n" );
		}
	}
	
	if( USBD.status == USBD_READY )     //�豸��ʼ�������
	{
		//�����豸��Ϣ�ṹ�壬���ж϶˵㣬�����䷢��IN��
		for( i=0;i!=USBD.itfmount;i++ )
		{
			for(j=0;j!=USBD.itf[i].edpmount;j++)
			{
				if((USBD.itf[i].edp[j].attr == 0x03) && (USBD.itf[i].edp[j].edpnum & 0x80) )  //�ж��ϴ��˵�
				{
					res = CH375InTrans( USBD.itf[i].edp[j].edpnum & 0x0F ,UserBuffer,&l,0 );     //�Զ˵㷢IN��,NAK������
					if( res == USB_INT_SUCCESS )
					{
						//printf("USB����ɹ�\r\n");
						if(l == 4)	//����ǲ���4���ֽڣ���׼��꣩
						{
							send_buff[0]=UserBuffer[0];
							send_buff[1]=UserBuffer[1];
							send_buff[2]=UserBuffer[2];
							send_buff[3]=UserBuffer[3];
							send_flag = 1;
						}
					}
				}
			}
		}
	}
}

void Check_Key(void)
{
	//������� + ��ʱ����
	
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

void Check_LED(void)
{
	static u8 counter_led = 0;
	
	switch(Led_flicker_Mode)
	{
		case 0:
			STM_EVAL_LEDOff(LED1);	//�ر�LED
		break;
		
		case 1:
			counter_led++;
			if( counter_led % 1 == 0 )	//50ms�任һ��
			{
				STM_EVAL_LEDToggle(LED1);
			}
			if(counter_led >= 2)		//ֻ������0.1s
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
	
	//Init Timer3 for us delay
	delay_init();
	
	/* Init Systick */
	SysTick_Init();	//Init and enable Systick.
	
	/* Init other peripheral */
	STM_EVAL_USART1_Init();	//Init Usart1 in EXIT mode
	STM_EVAL_LED1_Init();	//LED1 Port Init
	
	STM_EVAL_LED234_Init();	//LED234 Port Init
	STM_EVAL_PBInit();		//PUSH BUTTON Port Init
	
	/* Init communitcation port with slave */
	STM_EVAL_USART3_Init();	//Init Usart3 in EXIT mode
	
	/* Init CH375 */
	uint8_t res;
	res = mInitCH375Host();
	if( res!=USB_INT_SUCCESS)
	{
		printf("CH375��ʼ������\r\n");
		while(1);
	}
	CH375InitSysVar();         //�ϵ��ʼ���豸��ϢĬ��ֵ
	printf("CH375��ʼ���ɹ�\r\n");
	
	while (1)
	{
		//CH375
		if(send_flag != 1)	//��һ֡�Ѿ�����
		{
			Check_CH375();
		}
		
		//KEY
		if(Systick_5ms >= 5)
		{
			Systick_5ms = 0;
			
			Check_Key();
		}
		
		//LED
		if(Systick_50ms >= 50)
		{
			Systick_50ms = 0;
			
			Check_LED();
		}
		
		//USB�������� �� PS/2���յ�����
		if(bDeviceState == CONFIGURED && send_flag == 1)
		{
			//Usart_Mouse_Send(send_buff[0],send_buff[1],send_buff[2],send_buff[3]);			//���ڷ��ͳ�ȥ
			Usb_Mouse_Send(send_buff[0],send_buff[1],send_buff[2],send_buff[3]);				//USB���ͳ�ȥ
			Led_flicker_Mode = 1; 																//ָʾ����˸
			send_flag = 0;
		}
	}
}

/**
  * Function Name  : USART1_IRQHandler
  * Description    : This function handles Usart1 interrupt request.
					 �Ե��Եĵ��Զ˿�
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
  * Function Name  : USART3_IRQHandler
  * Description    : This function handles Usart3 interrupt request.
					 ���ӻ�ͨ�Ŷ˿�
  * Input          : None
  * Output         : None
  * Return         : None
  */
void USART3_IRQHandler(void)
{
	uint8_t ch;
	
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
	{ 	
	    //ch = USART3->DR;
		ch = USART_ReceiveData(USART3);
		
//		//�ѽ��յ��Ķ�������ȥ
//		while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);	/* �ȴ�������� */
//		USART_SendData(USART3, (uint8_t) ch);							/* ����һ���ֽ����ݵ�USART3 */
		
	}
}

/**
  * @brief  ����ϵͳ�δ�ʱ�� SysTick
  * @param  ��
  * @retval ��
  */
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


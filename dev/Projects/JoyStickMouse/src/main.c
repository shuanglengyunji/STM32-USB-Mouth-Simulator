/**
  ******************************************************************************
  * @file    main.c
  * @author  Liu Han
  * @brief   Joystick Mouse
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "hw_config.h"
#include "interface.h"     //底层接口函数
#include "HOST_SYS.H"      //主机操作函数
#include <stdio.h>
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
u32 Systick_5ms = 0;		//KEY
u32 Systick_50ms = 0;		//LED
u8 Led_flicker_Mode = 0;	//LED的闪烁模式

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
	
	if( CH375CheckConnect() == USBD_CONNECT )          /* 刚检测到一个设备接入，需要枚举 */
	{
		printf ( "设备连接成功\r\n" );
		
		//开始枚举操作
		
		printf ( "开始枚举\r\n" );
		
		/* 总线复位 */
		res = CH375BusReset();                   
		if( res != USB_INT_SUCCESS ) 
		{
			printf("Bus Reset Erro\n");
		}
		else
		{
			printf ( "总线复位成功\r\n" );
		}
		
		delay_ms( 50 );                          /* 等待设备稳定 */

		/* 获取设备描述符 */			
		res = CH375GetDeviceDesc( UserBuffer,&l); 
		if( res == USB_INT_SUCCESS )
		{
			printf ( "获取设备描述符成功\r\n" );
			for( i = 0; i < l; i++ )
			{
				printf("0x%02x ",(uint16_t)UserBuffer[i]);
			}
			printf ("\r\n");
		}
		else 
		{
			printf ( "获取设备描述符失败\r\n" );
			printf("Get Device Descr Erro:0x%02x\n",(uint16_t)res );
		}
		
		/* 设置地址 */
		res = CH375SetDeviceAddr( 2 );
		if( res!= USB_INT_SUCCESS )
		{
			printf ( "设置地址失败\r\n" );
			printf ("Set Addr Erro:0x%02x\n",(uint16_t)res );	
		}
		else
		{
			printf ( "设置地址成功\r\n" );
		}
		
		/* 获取配置描述符 */
		res = CH375GetConfDesc( UserBuffer,&l); 
		if( res== USB_INT_SUCCESS )
		{
			printf ( "获取配置描述符成功\r\n" );
			for( i = 0; i < l; i++ )
			{
				printf("0x%02x ",(uint16_t)UserBuffer[i]);
			}
			printf ("\r\n");					
		}
		else 
		{
			printf ( "获取配置描述符失败\r\n" );
			printf ("Get Conf Descr Erro:0x%02x\n",(uint16_t)res );	
		}			
		
		/* 设置配置 */
		res = CH375SetDeviceConf( 1 );
		if( res != USB_INT_SUCCESS ) 
		{
			printf ( "设置配置失败\r\n" );
			printf("Set Config Erro\n");
		}
		else
		{
			printf ( "设置配置成功\r\n" );
		}
	}
	
	if( USBD.status == USBD_READY )     //设备初始化已完成
	{
		//根据设备信息结构体，找中断端点，并对其发送IN包
		for( i=0;i!=USBD.itfmount;i++ )
		{
			for(j=0;j!=USBD.itf[i].edpmount;j++)
			{
				if((USBD.itf[i].edp[j].attr == 0x03) && (USBD.itf[i].edp[j].edpnum & 0x80) )  //中断上传端点
				{
					res = CH375InTrans( USBD.itf[i].edp[j].edpnum & 0x0F ,UserBuffer,&l,0 );     //对端点发IN包,NAK不重试
					if( res == USB_INT_SUCCESS )
					{
						//printf("USB传输成功\r\n");
						if(l == 4)	//检查是不是4个字节（标准鼠标）
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
	//按键检测 + 延时防抖
	
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
		printf("CH375初始化错误\r\n");
		while(1);
	}
	CH375InitSysVar();         //上电初始化设备信息默认值
	printf("CH375初始化成功\r\n");
	
	while (1)
	{
		//CH375
		if(send_flag != 1)	//上一帧已经发出
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
		
		//USB工作正常 且 PS/2接收到数据
		if(bDeviceState == CONFIGURED && send_flag == 1)
		{
			//Usart_Mouse_Send(send_buff[0],send_buff[1],send_buff[2],send_buff[3]);			//串口发送出去
			Usb_Mouse_Send(send_buff[0],send_buff[1],send_buff[2],send_buff[3]);				//USB发送出去
			Led_flicker_Mode = 1; 																//指示灯闪烁
			send_flag = 0;
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
  * Function Name  : USART3_IRQHandler
  * Description    : This function handles Usart3 interrupt request.
					 主从机通信端口
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
		
//		//把接收到的东西发回去
//		while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);	/* 等待发送完毕 */
//		USART_SendData(USART3, (uint8_t) ch);							/* 发送一个字节数据到USART3 */
		
	}
}

/**
  * @brief  启动系统滴答定时器 SysTick
  * @param  无
  * @retval 无
  */
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


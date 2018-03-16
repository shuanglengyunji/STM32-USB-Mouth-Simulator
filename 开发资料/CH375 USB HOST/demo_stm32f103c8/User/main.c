/**
  ******************************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2013-xx-xx
  * @brief   串口中断接收测试
  ******************************************************************************
  * @attention
  *
  * 实验平台:野火 iSO STM32 开发板 
  * 论坛    :http://www.chuxue123.com
  * 淘宝    :http://firestm32.taobao.com
  *
  ******************************************************************************
  */ 
 
 
#include "stm32f10x.h"
#include "bsp_usart1.h"
#include "delay.h"

#include "interface.h"     //底层接口函数
#include "HOST_SYS.H"      //主机操作函数

uint8_t UserBuffer[256];

/**
  * @brief  主函数
  * @param  无
  * @retval 无
  */
int main(void)
{
	delay_init();
	
	/* USART1 配置模式为 115200 8-N-1，中断接收 */
	USART1_Config();
	NVIC_Configuration();
	printf("USART1初始化完毕\r\n");	
	 
	/* 初始化CH375 */
	uint8_t res,i,j;
	uint16_t l;
	res = mInitCH375Host();
	if( res!=USB_INT_SUCCESS)
	{
		printf("ch375 init erro\n");
		while(1);
	}
	CH375InitSysVar();         //上电初始化设备信息默认值
	printf("CH375初始化完毕\r\n");
	
	while(1)
	{
		if( CH375CheckConnect() == USBD_CONNECT )          /* 刚检测到一个设备接入，需要枚举 */
		{
			printf ( "Device Connect\n" );                         
			//开始枚举操作		
			res = CH375BusReset();                   /* 总线复位 */
			if( res != USB_INT_SUCCESS ) 
				printf("Bus Reset Erro\n");
			delay_ms( 50 );                          /* 等待设备稳定 */

			/* 获取设备描述符 */			
			res = CH375GetDeviceDesc( UserBuffer,&l); 
			if( res == USB_INT_SUCCESS )
			{				
				for( i = 0; i < l; i++ )
					printf("0x%02x ",(uint16_t)UserBuffer[i]);
				printf ("\n");
			}
			else printf("Get Device Descr Erro:0x%02x\n",(uint16_t)res );
			
			/* 设置地址 */
			res = CH375SetDeviceAddr( 2 );
			if( res!= USB_INT_SUCCESS )
				printf ("Set Addr Erro:0x%02x\n",(uint16_t)res );				
			
			/* 获取配置描述符 */
			res = CH375GetConfDesc( UserBuffer,&l); 
			if( res== USB_INT_SUCCESS )
			{							
				for( i = 0; i < l; i++ )
					printf("0x%02x ",(uint16_t)UserBuffer[i]);
				printf ("\n");					
			}
			else printf ("Get Conf Descr Erro:0x%02x\n",(uint16_t)res );			
			
			/* 设置配置 */
			res = CH375SetDeviceConf( 1 );
			if( res != USB_INT_SUCCESS ) printf("Set Config Erro\n");
				
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
							for( i = 0; i < l; i++ )
								printf("0x%02x ",(uint16_t)UserBuffer[i]);
							printf ("\n");						
						}
					}
				}
			}
		}
	}
}
/*********************************************END OF FILE**********************/

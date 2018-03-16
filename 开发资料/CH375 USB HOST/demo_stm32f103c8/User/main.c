/**
  ******************************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2013-xx-xx
  * @brief   �����жϽ��ղ���
  ******************************************************************************
  * @attention
  *
  * ʵ��ƽ̨:Ұ�� iSO STM32 ������ 
  * ��̳    :http://www.chuxue123.com
  * �Ա�    :http://firestm32.taobao.com
  *
  ******************************************************************************
  */ 
 
 
#include "stm32f10x.h"
#include "bsp_usart1.h"
#include "delay.h"

#include "interface.h"     //�ײ�ӿں���
#include "HOST_SYS.H"      //������������

uint8_t UserBuffer[256];

/**
  * @brief  ������
  * @param  ��
  * @retval ��
  */
int main(void)
{
	delay_init();
	
	/* USART1 ����ģʽΪ 115200 8-N-1���жϽ��� */
	USART1_Config();
	NVIC_Configuration();
	printf("USART1��ʼ�����\r\n");	
	 
	/* ��ʼ��CH375 */
	uint8_t res,i,j;
	uint16_t l;
	res = mInitCH375Host();
	if( res!=USB_INT_SUCCESS)
	{
		printf("ch375 init erro\n");
		while(1);
	}
	CH375InitSysVar();         //�ϵ��ʼ���豸��ϢĬ��ֵ
	printf("CH375��ʼ�����\r\n");
	
	while(1)
	{
		if( CH375CheckConnect() == USBD_CONNECT )          /* �ռ�⵽һ���豸���룬��Ҫö�� */
		{
			printf ( "Device Connect\n" );                         
			//��ʼö�ٲ���		
			res = CH375BusReset();                   /* ���߸�λ */
			if( res != USB_INT_SUCCESS ) 
				printf("Bus Reset Erro\n");
			delay_ms( 50 );                          /* �ȴ��豸�ȶ� */

			/* ��ȡ�豸������ */			
			res = CH375GetDeviceDesc( UserBuffer,&l); 
			if( res == USB_INT_SUCCESS )
			{				
				for( i = 0; i < l; i++ )
					printf("0x%02x ",(uint16_t)UserBuffer[i]);
				printf ("\n");
			}
			else printf("Get Device Descr Erro:0x%02x\n",(uint16_t)res );
			
			/* ���õ�ַ */
			res = CH375SetDeviceAddr( 2 );
			if( res!= USB_INT_SUCCESS )
				printf ("Set Addr Erro:0x%02x\n",(uint16_t)res );				
			
			/* ��ȡ���������� */
			res = CH375GetConfDesc( UserBuffer,&l); 
			if( res== USB_INT_SUCCESS )
			{							
				for( i = 0; i < l; i++ )
					printf("0x%02x ",(uint16_t)UserBuffer[i]);
				printf ("\n");					
			}
			else printf ("Get Conf Descr Erro:0x%02x\n",(uint16_t)res );			
			
			/* �������� */
			res = CH375SetDeviceConf( 1 );
			if( res != USB_INT_SUCCESS ) printf("Set Config Erro\n");
				
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

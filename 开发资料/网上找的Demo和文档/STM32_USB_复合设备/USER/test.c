//#include <stm32f10x_lib.h>
#include <stm32f10x_map.h>
#include <stm32f10x_nvic.h>      

#include "sys.h"
#include "delay.h"	   
#include "usb_lib.h"
#include "hw_config.h"
#include "usb_pwr.h"  

extern u8 EP1BUSY;			//�������ݷ���æ��־ 
extern u8 EP2BUSY;			//������ݷ���æ��־
extern u8 INIT_OK;
		  
int main(void)
{
	Stm32_Clock_Init(9);//ϵͳʱ������
	delay_init(72);		//��ʱ��ʼ��
	//USB����
	USB_Interrupts_Config();    
	Set_USBClock();  
	USB_Init();
	delay_ms(1000);			//�ȴ���ʼ�����   
	while(1)
	{
		if(EP2BUSY==0)
		{
			EP2BUSY = 1;
			Joystick_Send(1,1);
		}
	}
}


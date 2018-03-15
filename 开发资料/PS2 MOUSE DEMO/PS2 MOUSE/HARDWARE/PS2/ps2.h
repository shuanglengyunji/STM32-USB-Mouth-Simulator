#ifndef __PS2_H
#define __PS2_H	 
#include "delay.h"	   
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK miniSTM32������
//PS2 ��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2012/9/13
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////			   
//����ӿڶ���
//PS2���� 		  				    
#define PS2_SCL PAin(15)			//PA15
#define PS2_SDA PCin(5)				//PC5
//PS2���
#define PS2_SCL_OUT PAout(15)		//PA15
#define PS2_SDA_OUT PCout(5)		//PC5

//����PS2_SCL�������״̬.		  
#define PS2_SET_SCL_IN()  {GPIOA->CRH&=0X0FFFFFFF;GPIOA->CRH|=0X80000000;}
#define PS2_SET_SCL_OUT() {GPIOA->CRH&=0X0FFFFFFF;GPIOA->CRH|=0X30000000;}	  
//����PS2_SDA�������״̬.		  
#define PS2_SET_SDA_IN()  {GPIOC->CRL&=0XFF0FFFFF;GPIOC->CRL|=0X00800000;}
#define PS2_SET_SDA_OUT() {GPIOC->CRL&=0XFF0FFFFF;GPIOC->CRL|=0X00300000;} 

#define MOUSE    0X20 //���ģʽ
#define CMDMODE  0X00 //����ģʽ

//PS2_Status��ǰ״̬��־
//[5:4]:��ǰ������ģʽ;[7]:���յ�һ������
//[6]:У�����;[3:0]:�յ������ݳ���;	 
extern u8 PS2_Status;       //����Ϊ����ģʽ
extern u8 PS2_DATA_BUF[16]; //ps2���ݻ�����
extern u8 MOUSE_ID;

void PS2_Init(void);
u8 PS2_Send_Cmd(u8 cmd);
void PS2_Set_Int(u8 en);
u8 PS2_Get_Byte(void);
void PS2_En_Data_Report(void);  
void PS2_Dis_Data_Report(void);		  				    
#endif






















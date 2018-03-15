#ifndef __MOUSE_H
#define __MOUSE_H	 
#include "ps2.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK mini�SSTM32������
//��� ��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2012/9/13
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////
   	 
//HOST->DEVICE�����				  				   
#define PS_RESET    		0XFF //��λ���� ��Ӧ0XFA
#define RESEND	     		0XFE //�ٴη���
#define SET_DEFAULT	 		0XF6 //ʹ��Ĭ������ ��Ӧ0XFA
#define DIS_DATA_REPORT   	0XF5 //�������ݱ��� ��Ӧ0XFA
#define EN_DATA_REPORT    	0XF4 //ʹ�����ݱ��� ��Ӧ0XFA
#define SET_SAMPLE_RATE		0XF3 //���ò������� ��Ӧ0XFA
#define GET_DEVICE_ID       0XF2 //�õ��豸ID   ��Ӧ0XFA+ID
#define SET_REMOTE_MODE     0XF0 //���õ�REMOTEģʽ ��ӦOXFA
#define SET_WRAP_MODE       0XEE //���õ�WRAPģʽ ��Ӧ0XFA
#define RST_WRAP_MODE       0XEC //�ص�WRAP֮ǰ��ģʽ ��Ӧ0XFA
#define READ_DATA           0XEB //��ȡ���� ��Ӧ0XFA+λ�����ݰ�
#define SET_STREAM_MODE     0XEA //���õ�STREAMģʽ ��Ӧ0XFA
#define STATUS_REQUEST      0XE9 //����õ�״̬ ��Ӧ0XFA+3���ֽ�
#define SET_RESOLUTION      0XE8 //���÷ֱ��� ��ӦOXFA+��ȡ1���ֽ�+Ӧ��0XFA
#define SET_SCALING21       0XE7 //�������ű���Ϊ2:1 ��Ӧ0XFA
#define SET_SCALING11       0XE6 //�������ű���Ϊ1:1 ��Ӧ0XFA
//DEVICE->HOST��ָ��
#define ERROR	     		0XFC //����
//#define RESEND	     		0XFE //�ٴη���

#define LEFT_DOWN  0X01//�������
#define MID_DOWN   0X04//�м������
#define RIGHT_DOWN 0X02//�Ҽ�����
//���ṹ��
typedef struct
{
	short x_pos;//������
	short y_pos;//������
	short z_pos;//��������
	u8  bt_mask;//������ʶ,bit2�м��;bit1,�Ҽ�;bit0,���
} PS2_Mouse;
extern PS2_Mouse MouseX;	   
extern u8 MOUSE_ID;//���ID,0X00,��ʾ��׼���(3�ֽ�);0X03��ʾ��չ���(4�ֽ�)

u8 Init_Mouse(void); 
void Mouse_Data_Pro(void);
	   	 			    
#endif













